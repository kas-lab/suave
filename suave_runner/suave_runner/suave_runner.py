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

"""Run repeatable SUAVE experiment campaigns and collect their results."""

import asyncio
import csv
from datetime import datetime
import json
import multiprocessing
import os
from pathlib import Path
from queue import Empty
import random
import shlex
import signal
import subprocess
import threading
import time
import traceback

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch import logging as launch_logging
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String
from suave_monitor.thruster_monitor import read_thruster_events
from suave_runner.attempt_processes import confirm_stopped
from suave_runner.attempt_processes import register_session
from suave_runner.attempt_processes import stop_sessions
from suave_runner.bag_validation import validate_bag
from suave_runner.diagnose_batch import summary_lines
from suave_runner.run_artifacts import artifact_path
from suave_runner.run_artifacts import attempt_details
from suave_runner.run_artifacts import campaign_lock
from suave_runner.run_artifacts import finalize_attempt
from suave_runner.run_artifacts import recording_enabled
from suave_runner.run_artifacts import recover_attempts
from suave_runner.run_status import load_statuses
from suave_runner.run_status import start_status
from suave_runner.run_status import write_status
import yaml

MISSION_DONE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class ExperimentRunnerNode(Node):
    """Orchestrate simulation and adaptation experiments."""

    def __init__(self, **kwargs):
        """Initialize campaign configuration and process state."""
        super().__init__('suave_runner_node', **kwargs)

        # Declare parameters

        # Runner parameters
        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('random_interval', 5)
        self.declare_parameter('random_seed', 100)
        # List of JSON-encoded dicts
        self.declare_parameter('experiments', [''])
        self.declare_parameter('run_duration', 600)  # seconds
        self.declare_parameter('readiness_timeout', 10.0)
        self.declare_parameter('gui', False)  # whether to run GUI or not
        # whether to log experiment results or not
        self.declare_parameter('experiment_logging', False)
        # Resume an interrupted campaign from an existing result folder.
        self.declare_parameter('resume_result_path', '')
        self.declare_parameter('record_bags', True)
        self._active_attempt = None
        self._active_log_dir = None

        # Ardupilot parameters
        self.declare_parameter(
            'ardupilot_executable',
            'sim_vehicle.py -L RATBeach -v ArduSub --model=JSON')

        # Simulation parameters
        self.declare_parameter(
            'suave_simulation',
            'ros2 launch suave simulation.launch.py')
        self.declare_parameter('initial_pos_x', -17.0)
        self.declare_parameter('initial_pos_y', 2.0)
        self.declare_parameter('initial_pos_z', -18.5)
        self.declare_parameter('initial_pos_x_random_interval', [0.0, 0.0])
        self.declare_parameter('initial_pos_y_random_interval', [0.0, 0.0])
        self.declare_parameter('initial_pos_z_random_interval', [0.0, 0.0])

        # Mission parameters
        self.declare_parameter('mission_config_pkg', 'suave_missions')
        self.declare_parameter(
            'mission_config_file',
            'config/mission_config.yaml')

        self.declare_parameter('water_visibility_sec_shift', 0.0)
        self.declare_parameter(
            'water_visibility_sec_shift_random_interval', [
                0.0, 0.0])

        self.declare_parameter('thruster_events', [''])
        self.declare_parameter('thruster_events_random_interval', [0.0, 0.0])
        self.declare_parameter('enable_water_current', False)

        # Retrieve parameters
        self.ardupilot_executable = self.get_parameter(
            'ardupilot_executable').get_parameter_value().string_value

        # Simulation parameters
        self.suave_simulation_cmd = self.get_parameter(
            'suave_simulation').get_parameter_value().string_value
        self.initial_pos_x = self.get_parameter(
            'initial_pos_x').get_parameter_value().double_value
        self.initial_pos_y = self.get_parameter(
            'initial_pos_y').get_parameter_value().double_value
        self.initial_pos_z = self.get_parameter(
            'initial_pos_z').get_parameter_value().double_value
        self.initial_pos_x_random_interval = self.get_parameter(
            'initial_pos_x_random_interval'
        ).get_parameter_value().double_array_value
        self.initial_pos_y_random_interval = self.get_parameter(
            'initial_pos_y_random_interval'
        ).get_parameter_value().double_array_value
        self.initial_pos_z_random_interval = self.get_parameter(
            'initial_pos_z_random_interval'
        ).get_parameter_value().double_array_value

        self.initial_pos_x_array = []
        self.initial_pos_y_array = []
        self.initial_pos_z_array = []

        # Runner parameters
        self.random_seed = self.get_parameter(
            'random_seed').get_parameter_value().integer_value
        random.seed(self.random_seed)
        self.run_duration = self.get_parameter(
            'run_duration').get_parameter_value().integer_value
        self.readiness_timeout = self.get_parameter('readiness_timeout').value
        if self.readiness_timeout < 0:
            raise ValueError('readiness_timeout must be non-negative')
        self.gui = self.get_parameter('gui').get_parameter_value().bool_value
        self.experiment_logging = self.get_parameter(
            'experiment_logging').get_parameter_value().bool_value
        experiments_param = self.get_parameter(
            'experiments').get_parameter_value().string_array_value
        try:
            self.experiments = [json.loads(exp_str)
                                for exp_str in experiments_param]
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f"Invalid JSON in 'experiments' parameter: {e}")
            return

        if not self.experiments:
            self.get_logger().error(
                "Parameter 'experiments' must be a non-empty list of JSON "
                'objects.')
            return

        self.wv_sec_shift = self.get_parameter(
            'water_visibility_sec_shift').get_parameter_value().double_value
        self.wv_sec_shift_interval = self.get_parameter(
            'water_visibility_sec_shift_random_interval'
        ).get_parameter_value().double_array_value
        self.wv_sec_shift_array = []

        self.thruster_events = read_thruster_events(self.get_parameter(
            'thruster_events').get_parameter_value().string_array_value)
        self.thruster_events_interval = self.get_parameter(
            'thruster_events_random_interval'
        ).get_parameter_value().double_array_value
        self.thruster_events_array = []
        self.enable_water_current = self.get_parameter(
            'enable_water_current').get_parameter_value().bool_value

        self.terminate_flag = False
        self.processes_stop_events = []
        self.ardupilot_proc = None

        self.ardupilot_cmd = ['xvfb-run', '-a'] + \
            self.ardupilot_executable.split()
        if self.gui is True:
            self.ardupilot_cmd = self.ardupilot_cmd[2:]

        self._mission_done_event = threading.Event()
        self._mission_failed_event = threading.Event()
        self._mission_failure_reason = ''
        self._process_failure_queue = multiprocessing.Queue()
        self.create_subscription(
            Bool,
            'mission_metrics/done',
            self._mission_done_cb,
            MISSION_DONE_QOS,
        )
        self.create_subscription(
            String,
            'mission/control_failure',
            self._mission_failed_cb,
            10,
        )

        self.get_logger().info(
            f'Runner initialized for {len(self.experiments)} experiments.')

    def handle_termination(self):
        """Request campaign termination and unblock mission waiting."""
        self.get_logger().warn('Signal received. Cleaning up...')
        self.terminate_flag = True
        self._mission_done_event.set()  # unblock any waiting run

    def _mission_done_cb(self, msg: Bool):
        if msg.data:
            self._mission_done_event.set()

    def _mission_failed_cb(self, msg: String):
        self._mission_failure_reason = msg.data
        self._mission_failed_event.set()

    @staticmethod
    def _record_process_exit(stop_event, failure_queue, event, _context):
        if stop_event.is_set() or event.returncode == 0:
            return None

        failure_queue.put({
            'process_name': event.process_name,
            'pid': event.pid,
            'returncode': event.returncode,
            'cmd': ' '.join(map(str, event.cmd)),
        })
        return None

    def _drain_process_failures(self):
        while True:
            try:
                self._process_failure_queue.get_nowait()
            except Empty:
                return

    def _get_process_failure(self):
        try:
            return self._process_failure_queue.get_nowait()
        except Empty:
            pass

        if (self.ardupilot_proc is not None and
                self.ardupilot_proc.poll() not in (None, 0)):
            return {
                'process_name': 'ArduPilot',
                'pid': self.ardupilot_proc.pid,
                'returncode': self.ardupilot_proc.returncode,
                'cmd': ' '.join(self.ardupilot_cmd),
            }

        for process, stop_event, launch_name in self.processes_stop_events:
            if (not stop_event.is_set() and process.exitcode is not None and
                    process.exitcode != 0):
                return {
                    'process_name': f'{launch_name} launch service',
                    'pid': process.pid,
                    'returncode': process.exitcode,
                    'cmd': '',
                }
        return None

    @staticmethod
    def _format_process_failure(failure):
        details = (
            f"{failure['process_name']} (pid {failure['pid']}) exited with "
            f"code {failure['returncode']}")
        if failure.get('cmd'):
            details += f"; command: {failure['cmd']}"
        return details

    def _wait_for_startup(self, duration):
        """Wait between launch stages while monitoring early failures."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.terminate_flag:
                raise RuntimeError('Termination requested during startup')
            failure = self._get_process_failure()
            if failure is not None:
                raise RuntimeError(self._format_process_failure(failure))
            time.sleep(min(0.2, deadline - time.monotonic()))

    def start_launch_process(
            self, launch_description: LaunchDescription, log_dir: Path,
            launch_name: str):
        """Start a ROS launch description in an isolated child process."""
        stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=self._run_launchfile,
            args=(stop_event, launch_description, log_dir, launch_name,
                  self._process_failure_queue),
        )
        process.start()
        return process, stop_event

    def _run_launchfile(
            self, stop_event, launch_description, log_dir, launch_name,
            failure_queue):
        # Establish durable ownership before any launch action spawns children.
        register_session(log_dir)
        # Ignore SIGINT in this process
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ['ROS_LOG_DIR'] = str(log_dir)
        launch_logging.reset()
        launch_logging.launch_config.log_dir = str(log_dir)

        process_exit_handler = RegisterEventHandler(
            OnProcessExit(
                on_exit=lambda event, context: self._record_process_exit(
                    stop_event, failure_queue, event, context)))
        monitored_launch_description = LaunchDescription([
            process_exit_handler,
            launch_description,
        ])

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        launch_service = LaunchService()
        launch_service.include_launch_description(
            monitored_launch_description)

        async def runner():
            await launch_service.run_async()

        async def shutdown_launch_service(service):
            await service.shutdown()
        # Start launch service as background task
        task = loop.create_task(runner())

        def stop_checker():
            stop_event.wait()
            loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(
                    shutdown_launch_service(launch_service)))

        threading.Thread(target=stop_checker, daemon=True).start()

        try:
            loop.run_until_complete(task)
        except BaseException:
            failure_log = log_dir / 'launcher_exception.log'
            with open(failure_log, 'a') as output:
                traceback.print_exc(file=output)
            if not stop_event.is_set():
                failure_queue.put({
                    'process_name': f'{launch_name} launch service',
                    'pid': os.getpid(),
                    'returncode': 1,
                    'cmd': '',
                })
            raise
        finally:
            loop.close()

    def shutdown_all_launch_processes(self):
        """Request shutdown of every tracked ROS launch process."""
        for process, stop_event, _launch_name in self.processes_stop_events:
            if process.is_alive():
                stop_event.set()
                process.join(timeout=30)
            if process.is_alive():
                # Do not forget the handle or move artifacts under live writers.
                raise RuntimeError(
                    f'Launch process {process.pid} did not exit cleanly; '
                    'attempt remains pending for recovery')
        self.processes_stop_events.clear()

    def shutdown_all_processes(self, log_dir=None):
        """Stop ArduPilot, ROS launches, and orphans in their sessions."""
        try:
            self.terminate_process(self.ardupilot_proc, 'ArduPilot')
            self.shutdown_all_launch_processes()
            log_dir = log_dir or self._active_log_dir
            if log_dir is not None:
                # Launch only signals its direct children; forked servers
                # such as Gazebo survive in the session until stopped here.
                stop_sessions(log_dir)
        finally:
            ardupilot_log = getattr(self, '_ardupilot_log', None)
            if ardupilot_log:
                ardupilot_log.close()
                self._ardupilot_log = None
        self.get_logger().info('All processes terminated.')

    def terminate_process(self, proc, name):
        """Terminate a running subprocess and its process group."""
        if proc and proc.poll() is None:
            self.get_logger().info(
                f'Terminating {name} process group {os.getpgid(proc.pid)}...')
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    def create_experiment_folder(self):
        """Return a resumed or newly created campaign result directory."""
        resume_path = self.get_parameter(
            'resume_result_path').get_parameter_value().string_value
        if resume_path:
            result_path = Path(resume_path).expanduser()
            self.get_logger().info(f'Resuming campaign from {result_path}')
        else:
            result_path_ = self.get_parameter(
                'result_path').get_parameter_value().string_value
            result_path = Path(result_path_).expanduser() / \
                datetime.now().strftime('%Y_%m_%d_%H-%M-%S')
        if not result_path.is_dir():
            result_path.mkdir(parents=True)
        return result_path

    def initialize_experiment(self, experiment, exp_idx):
        """Extract normalized launch and result settings for an experiment."""
        exp_launch = experiment.get('experiment_launch')
        num_runs = experiment.get('num_runs', 1)
        adaptation_manager = experiment.get('adaptation_manager', '')
        mission_type = experiment.get('mission_name', f'mission_{exp_idx}')

        result_filename = f'{adaptation_manager}_{mission_type}'

        return exp_launch, num_runs, result_filename, adaptation_manager

    def launch_ardupilot(self, log_path: Path):
        """Start ArduPilot and redirect its output to a run log."""
        self.get_logger().info('    Launching ArduPilot...')
        self._ardupilot_log = open(log_path, 'w')
        self.ardupilot_proc = subprocess.Popen(
            self.ardupilot_cmd,
            stdout=self._ardupilot_log,
            stderr=self._ardupilot_log,
            preexec_fn=os.setsid  # Launch in a new process group
        )
        self.get_logger().info(
            '    Sleeping 10 seconds before launching SUAVE simulation...')
        self._wait_for_startup(10)

    def launch_suave_simulation(self, run_idx, log_dir: Path):
        """Launch the SUAVE simulation for a configured run position."""
        suave_simulation_cmd_split = self.suave_simulation_cmd.split()
        sim_pkg = suave_simulation_cmd_split[2]
        sim_file = suave_simulation_cmd_split[3]
        sim_path = os.path.join(
            get_package_share_directory(sim_pkg),
            'launch',
            sim_file)

        sim_args = {}
        if len(suave_simulation_cmd_split) > 4:
            sim_args = {arg.split(':=')[0]: arg.split(':=')[1]
                        for arg in suave_simulation_cmd_split[4:]}
        sim_args['gui'] = 'true' if self.gui else 'false'
        if not self.experiment_logging:
            sim_args['silent'] = 'true'

        sim_args['x'] = str(self.initial_pos_x_array[run_idx])
        sim_args['y'] = str(self.initial_pos_y_array[run_idx])
        sim_args['z'] = str(self.initial_pos_z_array[run_idx])

        self.get_logger().info(
            f'    Launching SUAVE simulation from {sim_path} with args: '
            f'{sim_args}')
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_path),
            launch_arguments=list(sim_args.items())
        )
        sim_launch_ld = LaunchDescription()
        sim_launch_ld.add_action(sim_launch)
        sim_process, sim_stop_event = self.start_launch_process(
            sim_launch_ld, log_dir, 'simulation')
        self.processes_stop_events.append(
            (sim_process, sim_stop_event, 'simulation'))
        self.get_logger().info(
            '    Sleeping 10 seconds before launching next nodes...')
        self._wait_for_startup(10)

    def launch_experiment(
            self,
            exp_launch: str,
            result_path: str,
            result_filename: str,
            mission_config_file: str,
            log_dir: Path):
        """Launch the mission and managing system for one experiment run."""
        exp_launch_cmd_split = shlex.split(exp_launch)
        exp_pkg, exp_file = exp_launch_cmd_split[2], exp_launch_cmd_split[3]
        exp_path = os.path.join(
            get_package_share_directory(exp_pkg),
            'launch',
            exp_file)

        exp_args = {}
        if len(exp_launch_cmd_split) > 4:
            exp_args = dict(arg.split(':=', 1)
                            for arg in exp_launch_cmd_split[4:])
        exp_args['result_path'] = result_path
        exp_args['result_filename'] = result_filename
        exp_args['gui'] = 'true' if self.gui else 'false'
        if (self.enable_water_current and
                'enable_water_current' not in exp_args):
            exp_args['enable_water_current'] = 'true'
        if not self.experiment_logging:
            exp_args['silent'] = 'true'

        exp_args['mission_config'] = mission_config_file
        if self._active_attempt is not None:
            record = self._active_attempt
            exp_args['record_bags'] = str(record['recording_enabled']).lower()
            if record['recording_enabled']:
                exp_args['bag_output_path'] = str(artifact_path(
                    result_path, record['bag']['pending_path']))

        self.get_logger().info(
            f'    Launching Experiment from {exp_path} with args: {exp_args}')
        exp_launch_desc = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exp_path),
            launch_arguments=exp_args.items()
        )

        experiment_ld = LaunchDescription()
        experiment_ld.add_action(exp_launch_desc)
        experiment_process, experiment_stop_event = self.start_launch_process(
            experiment_ld, log_dir, 'experiment')
        self.processes_stop_events.append(
            (experiment_process, experiment_stop_event, 'experiment'))

    def _wait_for_readiness(self, manager, experiment):
        # BT XML uses change_mode even when behavior actions are enabled.
        # External managers have different interfaces; do not guess theirs.
        required = experiment.get('required_services')
        if required is None:
            required = [f'/{name}/change_mode' for name in (
                'f_maintain_motion', 'f_generate_search_path',
                'f_follow_pipeline')] if manager == 'bt' else []
            if manager == 'bt' and 'suave_bt_extended' in experiment.get(
                    'experiment_launch', ''):
                required.append('/generate_recharge_path/change_mode')
        if not isinstance(required, list) or any(
                not isinstance(name, str) or not name.startswith('/')
                for name in required):
            raise ValueError('required_services must be absolute service names')
        deadline = time.monotonic() + self.readiness_timeout
        while required:
            self._wait_for_startup(0.001)
            if self._mission_failed_event.is_set():
                raise RuntimeError(self._mission_failure_reason)
            available = dict(self.get_service_names_and_types())
            missing = [name for name in required if name not in available or (
                name.endswith('/change_mode') and
                'system_modes_msgs/srv/ChangeMode' not in available[name])]
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError('Missing required services: ' +
                                   ', '.join(missing))
            time.sleep(0.1)

    def run_experiments(self):
        """Execute campaigns under an exclusive lock and recover transactions."""
        result_path = self.create_experiment_folder()
        with campaign_lock(result_path):
            recover_attempts(result_path, validate_bag, confirm_stopped)
            self._run_experiments_locked(result_path)

    def _run_experiments_locked(self, result_path):
        mission_configs = self.generate_mission_config_files(result_path)
        for exp_idx, experiment in enumerate(self.experiments):
            exp_launch, num_runs, filename, manager = (
                self.initialize_experiment(experiment, exp_idx))
            if not exp_launch:
                continue
            enabled = recording_enabled(
                exp_launch, self.get_parameter('record_bags').value)
            self.get_logger().info(
                f'Running experiment {exp_idx + 1}/{len(self.experiments)} '
                f'for {num_runs} runs with {manager}')
            for run_idx in range(num_runs):
                if self.terminate_flag:
                    return
                run_id = f'run_{exp_idx}_{run_idx}'
                if (result_path / f'{run_id}.done').exists():
                    self.get_logger().info(
                        f'  Skipping {manager} run {run_idx + 1}/{num_runs} '
                        '(checkpoint found).')
                    continue
                mission = experiment.get('mission_name', f'mission_{exp_idx}')
                details = attempt_details(run_id, manager, mission, enabled)
                status_path, record = start_status(
                    result_path, exp_idx, run_idx, manager, mission,
                    mission_config=str(mission_configs[run_idx]),
                    experiment_launch=exp_launch,
                    simulation_launch=self.suave_simulation_cmd,
                    ardupilot_command=self.ardupilot_cmd,
                    random_seed=self.random_seed, **details)
                log_dir = artifact_path(result_path, record['log_dir'])
                log_dir.mkdir(parents=True, exist_ok=True)
                self._active_attempt = record
                self._active_log_dir = log_dir
                self._mission_done_event.clear()
                self._mission_failed_event.clear()
                self._mission_failure_reason = ''
                self._drain_process_failures()
                self.get_logger().info(
                    f'  Run {manager} {run_idx + 1}/{num_runs}')
                outcome, reason, failure_type = 'failed', '', ''
                stage = 'startup'
                try:
                    self.launch_ardupilot(log_dir / 'ardupilot.log')
                    self.launch_suave_simulation(run_idx, log_dir / 'simulation')
                    self.launch_experiment(
                        exp_launch, str(result_path), filename,
                        str(mission_configs[run_idx]), log_dir / 'experiment')
                    self._wait_for_readiness(manager, experiment)
                    stage = 'mission'
                    outcome, reason, failure_type = self._wait_for_mission()
                except Exception as error:
                    reason = str(error)
                    failure_type = (
                        'readiness_missing_service'
                        if stage == 'startup' and isinstance(error, TimeoutError)
                        else 'startup_failure' if stage == 'startup'
                        else 'runner_exception')
                    self.get_logger().error(reason)
                finally:
                    # If shutdown cannot be verified, persist why, leave the bag
                    # pending and propagate; never start another attempt.
                    try:
                        self.shutdown_all_processes(log_dir)
                    except Exception:
                        record.update(
                            execution_status='interrupted',
                            failure_type='shutdown_unconfirmed',
                            failure_reason='Launch shutdown was not confirmed')
                        write_status(status_path, record)
                        raise
                    finally:
                        self._active_attempt = None
                        self._active_log_dir = None

                if self.terminate_flag:
                    outcome, reason, failure_type = (
                        'interrupted', 'Termination requested', 'interrupted')
                if outcome == 'completed':
                    broken = self.detect_broken_run(log_dir)
                    if broken:
                        outcome, reason, failure_type = (
                            'failed', broken, 'broken_run')
                        self.record_broken_run(
                            result_path, exp_idx, run_idx, manager,
                            filename, broken)
                record.update(execution_status=outcome,
                              failure_reason=reason, failure_type=failure_type)
                write_status(status_path, record)
                finalize_attempt(result_path, status_path, record, validate_bag)
                self._log_run_summary(result_path, record)
                if self.terminate_flag:
                    return
                time.sleep(10)
            self.get_logger().info(f'Experiment {exp_idx + 1} summary:')
            for line in summary_lines(load_statuses(result_path)):
                self.get_logger().info(line)
        self.get_logger().info('All experiment runs completed or aborted.')

    def _wait_for_mission(self):
        self.get_logger().info('    Waiting for mission_metrics/done...')
        start = time.monotonic()
        while not self.terminate_flag:
            failure = self._get_process_failure()
            if failure is not None:
                return ('failed', self._format_process_failure(failure),
                        'process_died_before_completion')
            if self._mission_failed_event.is_set():
                return ('failed', self._mission_failure_reason,
                        'mission_control_failure')
            if self._mission_done_event.wait(timeout=0.2):
                return 'completed', '', ''
            if time.monotonic() - start > self.run_duration:
                return ('failed', 'Timeout waiting for mission_metrics/done.',
                        'mission_done_timeout')
        return 'interrupted', 'Termination requested', 'interrupted'

    def _log_run_summary(self, result_path, record):
        self.get_logger().info(
            f'  {record["run_id"]} {record["adaptation_manager"]}: '
            f'{record["status"]}; {record["failure_type"]} '
            f'{record["failure_reason"]}')
        for line in summary_lines(load_statuses(result_path)):
            if line.startswith('SYSTEMIC:'):
                self.get_logger().error(line)
            else:
                self.get_logger().info(line)

    def detect_broken_run(self, run_log_dir):
        """Return a reason string if the finished run hit a known bug, else ''."""
        return self._search_path_node_never_activated(run_log_dir)

    def _search_path_node_never_activated(self, run_log_dir):
        """Flag runs where the spiral search lifecycle node never activated."""
        experiment_dir = run_log_dir / 'experiment'
        # Match the node's own logger tag: other logs (e.g. the bag
        # recorder's topic list) mention the node name too.
        tag = '[f_generate_search_path_node]: '
        for log_path in sorted(experiment_dir.glob('python3_*.log')):
            try:
                text = log_path.read_text(errors='replace')
            except OSError:
                continue
            if (tag + 'on_configure() is called.' in text
                    and tag + 'on_activate() is called.' not in text):
                return ('f_generate_search_path_node configured but never '
                        'activated (system_modes mode transition lost)')
        return ''

    def record_broken_run(
            self, result_path, exp_idx, run_idx, manager, result_filename,
            reason):
        """Log a broken run and move its metrics row into a rejected file."""
        result_path = Path(result_path)
        record = {
            'run': f'run_{exp_idx}_{run_idx}',
            'adaptation_manager': manager,
            'reason': reason,
            'timestamp': datetime.now().isoformat(),
        }
        with open(result_path / 'rejected_runs.jsonl', 'a') as handle:
            handle.write(json.dumps(record) + '\n')

        results_csv = result_path / f'{result_filename}.csv'
        if not results_csv.is_file():
            return
        with open(results_csv, newline='') as handle:
            rows = list(csv.reader(handle))
        if len(rows) <= 1:
            return
        header = rows[0]
        rejected_row = rows[-1]
        with open(results_csv, 'w', newline='') as handle:
            csv.writer(handle).writerows(rows[:-1])
        rejected_csv = result_path / f'{result_filename}_rejected.csv'
        write_header = not rejected_csv.is_file()
        with open(rejected_csv, 'a', newline='') as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(header)
            writer.writerow(rejected_row)

    def generate_mission_config_files(self, result_path):
        """Generate per-run mission configurations with randomized events."""
        # Mission parameters
        mission_config_pkg = self.get_parameter(
            'mission_config_pkg').get_parameter_value().string_value
        mission_config_file = self.get_parameter(
            'mission_config_file').get_parameter_value().string_value
        mission_config_file = Path(get_package_share_directory(
            mission_config_pkg)) / mission_config_file

        mission_config_file_array = []
        for idx, wv_shift in enumerate(self.wv_sec_shift_array):
            # Load the original YAML
            with open(mission_config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Replace the value of watervi sibility sec shift
            observer_params = config[
                '/water_visibility_observer_node']['ros__parameters']
            observer_params['water_visibility_sec_shift'] = float(wv_shift)

            if len(self.thruster_events) > 0:
                thruster_params = config[
                    '/thruster_monitor']['ros__parameters']
                event_shift = self.thruster_events_array[idx]
                thruster_params['thruster_events'] = [
                    f'({event[0]},{event[1]},'
                    f'{float(event[2]) + float(event_shift)})'
                    for event in self.thruster_events
                ]

            # Create a new filename; reuse existing file when resuming
            new_file = result_path / f'mission_config_run{idx}.yaml'
            if not new_file.exists():
                with open(new_file, 'w') as f:
                    yaml.safe_dump(
                        config,
                        f,
                        default_flow_style=False,
                        indent=2,
                        sort_keys=False)

            mission_config_file_array.append(new_file)
        return mission_config_file_array

    def append_array_random_interval(
            self,
            array,
            value,
            random_interval,
            current_delta,
            min_delta,
            max_delta,
            i):
        """Append a value using a delta refreshed at fixed run intervals."""
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
        """Generate reproducible perturbations for all runs."""
        random_interval = self.get_parameter(
            'random_interval').get_parameter_value().integer_value

        # List of variables to randomize
        # Each tuple: (array, value, interval, current_delta)
        arrays = [
            (self.initial_pos_x_array, self.initial_pos_x,
             self.initial_pos_x_random_interval, 0.0),
            (self.initial_pos_y_array, self.initial_pos_y,
             self.initial_pos_y_random_interval, 0.0),
            (self.initial_pos_z_array, self.initial_pos_z,
             self.initial_pos_z_random_interval, 0.0),
            (self.wv_sec_shift_array, self.wv_sec_shift,
             self.wv_sec_shift_interval, 0.0),
            (self.thruster_events_array, 0,
             self.thruster_events_interval, 0.0),
        ]

        for exp_idx, experiment in enumerate(self.experiments):
            _, num_runs, _, _ = self.initialize_experiment(experiment, exp_idx)

            # Use a list for current_delta per variable
            current_deltas = [item[3] for item in arrays]

            for i in range(num_runs):
                for idx, (array, value, interval, _) in enumerate(arrays):
                    min_delta, max_delta = interval
                    current_deltas[idx] = self.append_array_random_interval(
                        array,
                        value,
                        random_interval,
                        current_deltas[idx],
                        min_delta,
                        max_delta,
                        i)


def main(args=None):
    """Run the SUAVE experiment campaign node."""
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = ExperimentRunnerNode()
    lc_node.randomize_experiments_configuration()
    executor.add_node(lc_node)

    try:
        run_experiment_task = executor.create_task(lc_node.run_experiments)
        executor.spin_until_future_complete(run_experiment_task)
        run_experiment_task.result()  # Propagate artifact/recovery failures.
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
