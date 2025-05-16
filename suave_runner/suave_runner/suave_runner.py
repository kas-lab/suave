import asyncio
import multiprocessing
import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import signal
import json
from datetime import datetime
import ament_index_python.packages

from launch import LaunchDescription
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node as LaunchNode



class ExperimentRunnerNode(Node):
    def __init__(self):
        super().__init__('experiment_runner_node')

        suave_missions_pkg_path = ament_index_python.packages.get_package_share_directory('suave_missions')
        mission_config_path = os.path.join(suave_missions_pkg_path, 'config', 'mission_config.yaml')

        # Declare parameters
        self.declare_parameter('ardupilot_executable', 'sim_vehicle.py -L RATBeach -v ArduSub --model=JSON')
        self.declare_parameter('suave_simulation', 'ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0')
        self.declare_parameter('experiments', rclpy.Parameter.Type.STRING_ARRAY)  # List of JSON-encoded dicts
        self.declare_parameter('mission_config', mission_config_path)  # path to .yaml file
        self.declare_parameter('run_duration', 600)  # seconds

        # Retrieve parameters
        self.ardupilot_executable = self.get_parameter('ardupilot_executable').get_parameter_value().string_value
        self.suave_simulation_cmd = self.get_parameter('suave_simulation').get_parameter_value().string_value
        self.mission_config_path = self.get_parameter('mission_config').get_parameter_value().string_value
        self.run_duration = self.get_parameter('run_duration').get_parameter_value().integer_value

        experiments_param = self.get_parameter('experiments').get_parameter_value().string_array_value
        try:
            self.experiments = [json.loads(exp_str) for exp_str in experiments_param]
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in 'experiments' parameter: {e}")
            rclpy.shutdown()
            return

        if not self.experiments:
            self.get_logger().error("Parameter 'experiments' must be a non-empty list of JSON objects.")
            rclpy.shutdown()
            return


        self.get_logger().info(f"Runner initialized for {len(self.experiments)} experiments.")
        self.terminate_flag = False

        signal.signal(signal.SIGINT, self.handle_termination)
        signal.signal(signal.SIGTERM, self.handle_termination)

        self.run_experiments()

    def handle_termination(self, signum, frame):
        self.get_logger().warn(f"Signal {signum} received. Cleaning up...")
        self.terminate_flag = True

    def start_launch_process(self, launch_description: LaunchDescription):
        stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=self._run_launchfile,
            args=(stop_event, launch_description),
            daemon=True
        )
        process.start()
        return process, stop_event

    def _run_launchfile(self, stop_event, launch_description):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_task = loop.create_task(launch_service.run_async())
        loop.run_until_complete(loop.run_in_executor(None, stop_event.wait))
        if not launch_task.done():
            asyncio.ensure_future(launch_service.shutdown(), loop=loop)
            loop.run_until_complete(launch_task)

    def shutdown_launch_process(self, process, stop_event):
        stop_event.set()
        process.join()

    def run_experiments(self):
        for exp_idx, experiment in enumerate(self.experiments):
            if self.terminate_flag:
                break

            exp_launch = experiment.get("experiment_launch")
            suave_base_launch = experiment.get("suave_base_launch")
            num_runs = experiment.get("num_runs", 1)
            adaptation_manager = experiment.get("adaptation_manager", "")
            mission_type = experiment.get("mission_name", f"mission_{exp_idx}")

            if not exp_launch or not suave_base_launch:
                self.get_logger().warn(f"Skipping experiment {exp_idx + 1}: Missing 'experiment_launch' or 'suave_base_launch'")
                continue

            date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_filename = f"{date_str}_{adaptation_manager}_{mission_type}"

            self.get_logger().info(f"Running experiment {exp_idx + 1}/{len(self.experiments)} for {num_runs} runs")

            for run_idx in range(num_runs):
                if self.terminate_flag:
                    break

                self.get_logger().info(f"  Run {run_idx + 1}/{num_runs}")

                # ardupilot_cmd = ['xvfb-run', '-a'] + self.ardupilot_executable.split()
                ardupilot_cmd = self.ardupilot_executable.split()
                # suave_base_cmd = ['ros2', 'launch'] + suave_base_launch.split()
                # experiment_cmd = ['ros2', 'launch'] + exp_launch.split()

                # metrics_cmd = [
                #     'ros2', 'run', 'suave_metrics', 'mission_metrics',
                #     '--ros-args',
                #     '-p', f'adaptation_manager:={adaptation_manager}',
                #     '-p', f'mission_name:={mission_type}',
                #     '-p', f'result_filename:={result_filename}',
                #     '--params-file', self.mission_config_path
                # ]

                metrics_node = LaunchNode(
                    package='suave_metrics',
                    executable='mission_metrics',
                    name='mission_metrics',
                    parameters=[self.mission_config_path, {
                        'adaptation_manager': adaptation_manager,
                        'mission_name': mission_type,
                        'result_filename': result_filename,
                    }]
                )

                try:
                    self.get_logger().info("    Launching ArduPilot...")
                    ardupilot_proc = subprocess.Popen(ardupilot_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    self.get_logger().info("    Sleeping 10 seconds before launching SUAVE simulation...")
                    time.sleep(10)

                    # simulation_proc = subprocess.Popen(self.suave_simulation_cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    # Launch ROS 2 simulation.launch.py using LaunchService
                    suave_simulation_cmd_split = self.suave_simulation_cmd.split()
                    sim_pkg, sim_file = suave_simulation_cmd_split[2], suave_simulation_cmd_split[3]
                    sim_path = os.path.join(get_package_share_directory(sim_pkg), 'launch', sim_file)
                    sim_args = {}
                    if len(suave_simulation_cmd_split) > 4:
                        sim_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in suave_simulation_cmd_split[4:]}

                    
                    self.get_logger().info(f"    Launching SUAVE simulation from {sim_path} with args: {sim_args}")
                    sim_launch = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(sim_path),
                        launch_arguments=list(sim_args.items())
                    )
                    sim_launch_ld = LaunchDescription()
                    sim_launch_ld.add_action(sim_launch)
                    sim_process, sim_stop_event = self.start_launch_process(sim_launch_ld)
                    # sim_ls = LaunchService()
                    # sim_ls.include_launch_description(sim_launch)
                    # self.get_logger().info("    Launching SUAVE simulation...")
                    # sim_task = sim_ls.run_async()
                    # if sim_task:
                    #     self.get_logger().info("SUAVE simulation task started successfully.")
                    # else:
                    #     self.get_logger().error("Failed to start SUAVE simulation task.")

                    # # Log additional information about the SUAVE simulation task
                    # try:
                    #     sim_ls_result = sim_task.result()
                    #     self.get_logger().info(f"SUAVE simulation task result: {sim_ls_result}")
                    # except Exception as e:
                    #     self.get_logger().error(f"Error while running SUAVE simulation task: {e}")

                    self.get_logger().info("    Sleeping 10 seconds before launching next nodes...")
                    time.sleep(10)

                    # suave_base_proc = subprocess.Popen(suave_base_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    # Launch suave_base
                    suave_base_cmd_split = suave_base_launch.split()
                    suave_base_pkg, suave_base_file = suave_base_cmd_split[2], suave_base_cmd_split[3]
                    suave_base_path = os.path.join(get_package_share_directory(suave_base_pkg), 'launch', suave_base_file)
                    suave_base_args = {}
                    if len(suave_base_cmd_split) > 4:
                        suave_base_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in suave_base_cmd_split[4:]}
                    
                    suave_base_launch_desc = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(suave_base_path),
                        launch_arguments=suave_base_args.items()
                    )

                    # experiment_proc = subprocess.Popen(experiment_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    # Launch experiment
                    exp_launch_cmd_split = exp_launch.split()
                    exp_pkg, exp_file = exp_launch_cmd_split[2], exp_launch_cmd_split[3]
                    exp_path = os.path.join(get_package_share_directory(exp_pkg), 'launch', exp_file)
                    
                    exp_args = {}
                    if len(exp_launch_cmd_split) > 4:
                        exp_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in exp_launch_cmd_split[4:]}

                    exp_launch_desc = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(exp_path),
                        launch_arguments=exp_args.items()
                    )


                    # Create and run a combined LaunchService for base + experiment
                    experiment_ld = LaunchDescription()
                    experiment_ld.add_action(metrics_node)
                    experiment_ld.add_action(suave_base_launch_desc)
                    experiment_ld.add_action(exp_launch_desc)
                    # experiment_ls = LaunchService()
                    # experiment_ls.include_launch_description(experiment_ld)
                    experiment_process, experiment_stop_event = self.start_launch_process(experiment_ld)
                    # experiment_ls.include_launch_description(suave_base_launch_desc)
                    # experiment_ls.include_launch_description(exp_launch_desc)
                    # experiment_ls.include_launch_description(metrics_node_ld)
                    # experiment_task = experiment_ls.run_async()

                    # metrics_proc = subprocess.Popen(metrics_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to launch processes: {e}")
                    break

                self.get_logger().info("    Waiting for /tmp/mission.done...")
                start_time = time.time()
                file_detected = False

                while not self.terminate_flag:
                    if os.path.exists("/tmp/mission.done"):
                        file_detected = True
                        try:
                            os.remove("/tmp/mission.done")
                            self.get_logger().info("    Run completed: /tmp/mission.done detected and removed.")
                        except Exception as e:
                            self.get_logger().warn(f"    Could not remove /tmp/mission.done: {e}")
                        break

                    if time.time() - start_time > self.run_duration:
                        self.get_logger().warn("    Timeout waiting for /tmp/mission.done.")
                        break

                    rclpy.spin_once(self, timeout_sec=1.0)

                # self.terminate_process(experiment_proc, "experiment")
                # self.terminate_process(metrics_proc, "mission_metrics")
                # self.terminate_process(suave_base_proc, "SUAVE base")
                # self.terminate_process(simulation_proc, "simulation")
                self.terminate_process(ardupilot_proc, "ArduPilot")
                self.shutdown_launch_process(sim_process, sim_stop_event)
                self.shutdown_launch_process(experiment_process, experiment_stop_event)
                # sim_ls.shutdown()

                # experiment_ls.shutdown()

                if self.terminate_flag:
                    self.get_logger().info("Termination requested. Stopping early.")
                    break

                self.get_logger().info(f"  Run {run_idx + 1} completed (success: {file_detected}).")

        self.get_logger().info("All experiment runs completed or aborted.")
        rclpy.shutdown()

    def terminate_process(self, proc, name):
        if proc and proc.poll() is None:
            self.get_logger().info(f"Terminating {name} process...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Forcing {name} process to stop.")
                proc.kill()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunnerNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
