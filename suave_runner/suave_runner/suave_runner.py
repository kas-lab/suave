# Copyright 2025 KAS-lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import multiprocessing
import threading
import subprocess
import time
import os
import signal
import json
import random
import yaml
from datetime import datetime
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.node import Node

from launch import LaunchDescription
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

from suave_monitor.thruster_monitor import read_thruster_events

class ExperimentRunnerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('suave_runner_node', **kwargs)
        random.seed(100) # set random seed

        # Declare parameters

        ## Runner parameters
        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('random_interval', 5)
        self.declare_parameter('experiments', [''])  # List of JSON-encoded dicts
        self.declare_parameter('run_duration', 600)  # seconds
        self.declare_parameter('gui', False)  # whether to run GUI or not
        self.declare_parameter('experiment_logging', False)  # whether to log experiment results or not
        
        ## Ardupilot parameters
        self.declare_parameter('ardupilot_executable', 'sim_vehicle.py -L RATBeach -v ArduSub --model=JSON')
        
        ## Simulation parameters
        self.declare_parameter('suave_simulation', 'ros2 launch suave simulation.launch.py')
        self.declare_parameter('initial_pos_x', -17.0)
        self.declare_parameter('initial_pos_y', 2.0)
        self.declare_parameter('initial_pos_z', -18.5)
        self.declare_parameter('initial_pos_x_random_interval', [0.0, 0.0])
        self.declare_parameter('initial_pos_y_random_interval', [0.0, 0.0])
        self.declare_parameter('initial_pos_z_random_interval', [0.0, 0.0])
        
        ## Mission parameters
        self.declare_parameter('mission_config_pkg', 'suave_missions')  # whether to log experiment results or not
        self.declare_parameter('mission_config_file', 'config/mission_config.yaml')  # whether to log experiment results or not

        self.declare_parameter('water_visibility_sec_shift', 0.0)
        self.declare_parameter('water_visibility_sec_shift_random_interval', [0.0, 0.0])

        self.declare_parameter('thruster_events', [''])
        self.declare_parameter('thruster_events_random_interval', [0.0, 0.0])

        # Retrieve parameters
        self.ardupilot_executable = self.get_parameter('ardupilot_executable').get_parameter_value().string_value
        
        ## Simulation parameters
        self.suave_simulation_cmd = self.get_parameter('suave_simulation').get_parameter_value().string_value
        self.initial_pos_x = self.get_parameter('initial_pos_x').get_parameter_value().double_value
        self.initial_pos_y = self.get_parameter('initial_pos_y').get_parameter_value().double_value
        self.initial_pos_z = self.get_parameter('initial_pos_z').get_parameter_value().double_value
        self.initial_pos_x_random_interval = self.get_parameter('initial_pos_x_random_interval').get_parameter_value().double_array_value
        self.initial_pos_y_random_interval = self.get_parameter('initial_pos_y_random_interval').get_parameter_value().double_array_value
        self.initial_pos_z_random_interval = self.get_parameter('initial_pos_z_random_interval').get_parameter_value().double_array_value
        
        self.initial_pos_x_array = []
        self.initial_pos_y_array = []
        self.initial_pos_z_array = []

        ## Runner parameters
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

        self.wv_sec_shift = self.get_parameter(
            'water_visibility_sec_shift').get_parameter_value().double_value
        self.wv_sec_shift_interval =  self.get_parameter(
            'water_visibility_sec_shift_random_interval').get_parameter_value().double_array_value
        self.wv_sec_shift_array =[]
        
        self.thruster_events = read_thruster_events(self.get_parameter(
            'thruster_events').get_parameter_value().string_array_value)
        self.thruster_events_interval =  self.get_parameter(
            'thruster_events_random_interval').get_parameter_value().double_array_value
        self.thruster_events_array = []

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

    def create_experiment_folder(self):
        result_path_ = self.get_parameter('result_path').get_parameter_value().string_value
        result_path = Path(result_path_).expanduser() / datetime.now().strftime("%Y_%m_%d_%H-%M-%S")

        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)
        return result_path

    def initialize_experiment(self, experiment, exp_idx):
        exp_launch = experiment.get("experiment_launch")
        num_runs = experiment.get("num_runs", 1)
        adaptation_manager = experiment.get("adaptation_manager", "")
        mission_type = experiment.get("mission_name", f"mission_{exp_idx}")

        result_filename = f"{adaptation_manager}_{mission_type}"

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

    def launch_suave_simulation(self, run_idx):
        suave_simulation_cmd_split = self.suave_simulation_cmd.split()
        sim_pkg, sim_file = suave_simulation_cmd_split[2], suave_simulation_cmd_split[3]
        sim_path = os.path.join(get_package_share_directory(sim_pkg), 'launch', sim_file)
        
        sim_args = {}
        if len(suave_simulation_cmd_split) > 4:
            sim_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in suave_simulation_cmd_split[4:]}
        sim_args['gui'] = 'true' if self.gui else 'false'
        if not self.experiment_logging:
            sim_args['silent'] = 'true'

        sim_args['x'] = str(self.initial_pos_x_array[run_idx])
        sim_args['y'] = str(self.initial_pos_y_array[run_idx])
        sim_args['z'] = str(self.initial_pos_z_array[run_idx])

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

    def launch_experiment(self, exp_launch: str, result_path: str, result_filename: str, mission_config_file: str):
        exp_launch_cmd_split = exp_launch.split()
        exp_pkg, exp_file = exp_launch_cmd_split[2], exp_launch_cmd_split[3]
        exp_path = os.path.join(get_package_share_directory(exp_pkg), 'launch', exp_file)
    
        exp_args = {}
        if len(exp_launch_cmd_split) > 4:
            exp_args = {arg.split(':=')[0]: arg.split(':=')[1] for arg in exp_launch_cmd_split[4:]}
        exp_args['result_path'] = result_path
        exp_args['result_filename'] = result_filename
        exp_args['gui'] = 'true' if self.gui else 'false'
        if not self.experiment_logging:
            exp_args['silent'] = 'true'
    
        exp_args['mission_config'] = mission_config_file

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
        result_path = self.create_experiment_folder()
        mission_config_file_array = self.generate_mission_config_files(
            result_path)
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
                    self.launch_suave_simulation(run_idx)
                    self.launch_experiment(
                        exp_launch, 
                        str(result_path), 
                        result_filename,
                        str(mission_config_file_array[run_idx]))
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

    def generate_mission_config_files(self, result_path):
        ## Mission parameters
        mission_config_pkg = self.get_parameter('mission_config_pkg').get_parameter_value().string_value
        mission_config_file = self.get_parameter('mission_config_file').get_parameter_value().string_value
        mission_config_file = Path(get_package_share_directory(mission_config_pkg)) / mission_config_file

        mission_config_file_array = []
        for idx, wv_shift in enumerate(self.wv_sec_shift_array):
            # Load the original YAML
            with open(mission_config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Replace the value of watervi sibility sec shift
            config['/water_visibility_observer_node']['ros__parameters']['water_visibility_sec_shift'] = float(wv_shift)
            
            config['/thruster_monitor']['ros__parameters']['thruster_events'] = [
                f"({event[0]},{event[1]},{float(event[2]) + float(self.thruster_events_array[idx])})"
                for event in self.thruster_events
            ]

            # Create a new filename
            new_file = result_path / f'mission_config_run{idx}.yaml'
            with open(new_file, 'w') as f:
                yaml.safe_dump(config, f, default_flow_style=False, indent=2, sort_keys=False)

            mission_config_file_array.append(new_file)
        return mission_config_file_array
    
    def append_array_random_interval(self, array, value, random_interval, current_delta, min_delta, max_delta, i):
        if i < len(array):
            return current_delta
        delta = current_delta
        if i % random_interval == 0:
            delta = random.uniform(
                min_delta, 
                max_delta
            )
        array.append(value + delta)
        return delta                                      
    
    def randomize_experiments_configuration(self):
        random_interval = self.get_parameter('random_interval').get_parameter_value().integer_value
        
        # List of variables to randomize
        # Each tuple: (array, value, interval, current_delta)
        arrays = [
            (self.initial_pos_x_array, self.initial_pos_x, self.initial_pos_x_random_interval, 0.0),
            (self.initial_pos_y_array, self.initial_pos_y, self.initial_pos_y_random_interval, 0.0),
            (self.initial_pos_z_array, self.initial_pos_z, self.initial_pos_z_random_interval, 0.0),
            (self.wv_sec_shift_array, self.wv_sec_shift, self.wv_sec_shift_interval, 0.0),
            (self.thruster_events_array, 0, self.thruster_events_interval, 0.0),
        ]
        
        for exp_idx, experiment in enumerate(self.experiments):
            _, num_runs, _, _ = self.initialize_experiment(experiment, exp_idx)
            
            # Use a list for current_delta per variable
            current_deltas = [item[3] for item in arrays]
            
            for i in range(num_runs):
                for idx, (array, value, interval, _) in enumerate(arrays):
                    min_delta, max_delta = interval
                    current_deltas[idx] = self.append_array_random_interval(
                        array, value, random_interval, current_deltas[idx], min_delta, max_delta, i
                    )

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = ExperimentRunnerNode()
    lc_node.randomize_experiments_configuration()
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
