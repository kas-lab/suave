import asyncio
import multiprocessing
import threading
import rclpy
import rclpy.executors
from rclpy.node import Node
import subprocess
import time
import os
import signal
import json
from datetime import datetime

from launch import LaunchDescription
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


class ExperimentRunnerNode(Node):
    def __init__(self):
        super().__init__('suave_runner_node')

        # Declare parameters
        self.declare_parameter('ardupilot_executable', 'sim_vehicle.py -L RATBeach -v ArduSub --model=JSON')
        self.declare_parameter('suave_simulation', 'ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0')
        self.declare_parameter('experiments', rclpy.Parameter.Type.STRING_ARRAY)  # List of JSON-encoded dicts
        self.declare_parameter('run_duration', 600)  # seconds
        self.declare_parameter('gui', False)  # whether to run GUI or not
        self.declare_parameter('experiment_logging', False)  # whether to log experiment results or not

        # Retrieve parameters
        self.ardupilot_executable = self.get_parameter('ardupilot_executable').get_parameter_value().string_value
        self.suave_simulation_cmd = self.get_parameter('suave_simulation').get_parameter_value().string_value
        self.run_duration = self.get_parameter('run_duration').get_parameter_value().integer_value
        self.gui = self.get_parameter('gui').get_parameter_value().bool_value
        self.experiment_logging = self.get_parameter('experiment_logging').get_parameter_value().bool_value
        experiments_param = self.get_parameter('experiments').get_parameter_value().string_array_value
        try:
            self.experiments = [json.loads(exp_str) for exp_str in experiments_param]
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON in 'experiments' parameter: {e}")
            return

        if not self.experiments:
            self.get_logger().error("Parameter 'experiments' must be a non-empty list of JSON objects.")
            return

        self.terminate_flag = False
        self.processes_stop_events = []

        self.ardupilot_cmd = ['xvfb-run', '-a'] + self.ardupilot_executable.split()
        if self.gui is True:
            self.ardupilot_cmd = self.ardupilot_cmd[2:]

        self.get_logger().info(f"Runner initialized for {len(self.experiments)} experiments.")

    def handle_termination(self):
        self.get_logger().warn(f"Signal received. Cleaning up...")
        self.terminate_flag = True

    def start_launch_process(self, launch_description: LaunchDescription):
        stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=self._run_launchfile,
            args=(stop_event, launch_description),
        )
        process.start()
        return process, stop_event

    def _run_launchfile(self, stop_event, launch_description):
        # Ignore SIGINT in this process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        
        async def runner():
            await launch_service.run_async()
        
        async def shutdown_launch_service(service):
            await service.shutdown()
        # Start launch service as background task
        task = loop.create_task(runner())

        def stop_checker():
            stop_event.wait()
            loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(shutdown_launch_service(launch_service))
            )

        threading.Thread(target=stop_checker, daemon=True).start()

        try:
            loop.run_until_complete(task)
        finally:
            loop.close()


    def shutdown_all_launch_processes(self):
        for process, stop_event in self.processes_stop_events:
            if process.is_alive():
                stop_event.set()
                process.join(timeout=10)
            if process.is_alive():
                self.get_logger().warn(f"Launch process {process.pid} did not exit cleanly.")
        self.processes_stop_events.clear()
        
    def kill_gz_sim(self):
        try:
            subprocess.run(["pkill", "-f", "gz sim"], check=False)
        except Exception as e:
            print(f"Error killing gz sim processes: {e}")
    
    def shutdown_all_processes(self):
        self.terminate_process(self.ardupilot_proc, "ArduPilot")
        self.shutdown_all_launch_processes()
        self.kill_gz_sim()
        self.get_logger().info("All processes terminated.")

    def terminate_process(self, proc, name):
        if proc and proc.poll() is None:
            self.get_logger().info(f"Terminating {name} process group {os.getpgid(proc.pid)}...")
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    def remove_done_file(self):
        try:
            if os.path.exists("/tmp/mission.done"):
                os.remove("/tmp/mission.done")
                self.get_logger().info("    /tmp/mission.done detected and removed.")
        except Exception as e:
            self.get_logger().warn(f"    Could not remove /tmp/mission.done: {e}")

    def initialize_experiment(self, experiment, exp_idx):
        exp_launch = experiment.get("experiment_launch")
        num_runs = experiment.get("num_runs", 1)
        adaptation_manager = experiment.get("adaptation_manager", "")
        mission_type = experiment.get("mission_name", f"mission_{exp_idx}")

        date_str = datetime.now().strftime("%Y_%m_%d_%H-%M-%S")
        result_filename = f"{date_str}_{adaptation_manager}_{mission_type}"

        return exp_launch, num_runs, result_filename, adaptation_manager

    def launch_ardupilot(self):
        self.get_logger().info("    Launching ArduPilot...")
        ardupilot_proc_log = subprocess.DEVNULL
        if self.experiment_logging:
            ardupilot_proc_log = subprocess.PIPE

        self.ardupilot_proc = subprocess.Popen(
            self.ardupilot_cmd,
            stdout=ardupilot_proc_log,
            stderr=ardupilot_proc_log,
            preexec_fn=os.setsid  # Launch in a new process group
        )
        self.get_logger().info("    Sleeping 10 seconds before launching SUAVE simulation...")
        time.sleep(10)

    def launch_suave_simulation(self):
        suave_simulation_cmd_split = self.suave_simulation_cmd.split()
        sim_pkg, sim_file = suave_simulation_cmd_split[2], suave_simulation_cmd_split[3]
        sim_path = os.path.join(get_package_share_directory(sim_pkg), 'launch', sim_file)
        
        sim_args = {}
        if len(suave_simulation_cmd_split) > 4:
            sim_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in suave_simulation_cmd_split[4:]}
        sim_args['gui'] = 'true' if self.gui else 'false'
        if not self.experiment_logging:
            sim_args['silent'] = 'true'

        self.get_logger().info(f"    Launching SUAVE simulation from {sim_path} with args: {sim_args}")
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_path),
            launch_arguments=list(sim_args.items())
        )
        sim_launch_ld = LaunchDescription()
        sim_launch_ld.add_action(sim_launch)
        sim_process, sim_stop_event = self.start_launch_process(sim_launch_ld)
        self.processes_stop_events.append((sim_process, sim_stop_event))
        self.get_logger().info("    Sleeping 10 seconds before launching next nodes...")
        time.sleep(10)

    def launch_experiment(self, exp_launch, result_filename):
        exp_launch_cmd_split = exp_launch.split()
        exp_pkg, exp_file = exp_launch_cmd_split[2], exp_launch_cmd_split[3]
        exp_path = os.path.join(get_package_share_directory(exp_pkg), 'launch', exp_file)
    
        exp_args = {}
        if len(exp_launch_cmd_split) > 4:
            exp_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in exp_launch_cmd_split[4:]}
        exp_args['result_filename'] = result_filename
        exp_args['gui'] = 'true' if self.gui else 'false'
        if not self.experiment_logging:
            exp_args['silent'] = 'true'
    
        self.get_logger().info(f"    Launching Experiment from {exp_path} with args: {exp_args}")
        exp_launch_desc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exp_path),
            launch_arguments=exp_args.items()
        )
    
        experiment_ld = LaunchDescription()
        experiment_ld.add_action(exp_launch_desc)
        experiment_process, experiment_stop_event = self.start_launch_process(experiment_ld)
        self.processes_stop_events.append((experiment_process, experiment_stop_event))

    def run_experiments(self):
        #TODO: get datetime here, instead of in initialize_experiment
        for exp_idx, experiment in enumerate(self.experiments):
            if self.terminate_flag:
                break

            self.remove_done_file()

            exp_launch, num_runs, result_filename, adaptation_manager = self.initialize_experiment(experiment, exp_idx)

            if not exp_launch:
                self.get_logger().warn(f"Skipping experiment {exp_idx + 1}: Missing 'experiment_launch'")
                continue

            self.get_logger().info(f"Running experiment {exp_idx + 1}/{len(self.experiments)} for {num_runs} runs with {adaptation_manager}")

            for run_idx in range(num_runs):
                if self.terminate_flag:
                    break

                self.get_logger().info(f"  Run {adaptation_manager} {run_idx + 1}/{num_runs}")

                try:
                    self.launch_ardupilot()
                    self.launch_suave_simulation()
                    self.launch_experiment(exp_launch, result_filename)
                except Exception as e:
                    self.get_logger().error(f"Failed to launch processes: {e}")
                    break

                self.get_logger().info("    Waiting for /tmp/mission.done...")
                start_time = time.time()
                file_detected = False

                while not self.terminate_flag:
                    if os.path.exists("/tmp/mission.done"):
                        file_detected = True
                        self.remove_done_file()
                        break

                    if time.time() - start_time > self.run_duration:
                        self.get_logger().warn("    Timeout waiting for /tmp/mission.done.")
                        break

                    self.executor.spin_once(timeout_sec=1.0)

                self.shutdown_all_processes()

                if self.terminate_flag:
                    self.get_logger().info("Termination requested. Stopping early.")
                    break

                self.get_logger().info(f"  Run {run_idx + 1} completed (success: {file_detected}).")
                time.sleep(10)
        
        self.get_logger().info("All experiment runs completed or aborted.")

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = ExperimentRunnerNode()
    executor.add_node(lc_node)
    
    try:
        run_experiment_task = executor.create_task(lc_node.run_experiments)
        executor.spin_until_future_complete(run_experiment_task)
    except KeyboardInterrupt:
        print(' Ctrl+C received. Triggering shutdown...')
        lc_node.handle_termination()
        lc_node.shutdown_all_processes()
    finally:
        if rclpy.ok():
            executor.shutdown()
            lc_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    main()
