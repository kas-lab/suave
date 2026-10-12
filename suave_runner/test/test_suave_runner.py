# Copyright 2025 Gustavo Rezende Silva
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

import json
from pathlib import Path
from queue import Empty
from queue import Queue
import shutil
import socket
import time
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import String

from suave_runner.run_artifacts import attempt_details
from suave_runner.run_status import open_runs
from suave_runner.run_status import start_attempt
from suave_runner.suave_runner import check_no_stale_simulation
from suave_runner.suave_runner import ExperimentRunnerNode
from suave_runner.suave_runner import port_listening
from suave_runner.suave_runner import ReadinessTimeout
from suave_runner.suave_runner import StaleSimulationError
from suave_runner.suave_runner import udp_port_bound
from suave_tools.runs import load_runs
from suave_tools.runs import write_runs
import yaml


def _minimal_runner_params():
    return [
        Parameter('experiments', Parameter.Type.STRING_ARRAY, [
            _experiment('bt', 1)
        ])
    ]


def _experiment(manager, num_runs):
    return json.dumps({
        'experiment_launch': (
            'ros2 launch suave_bringup mission.launch.py '
            f'adaptation_manager:={manager}'),
        'num_runs': num_runs,
        'adaptation_manager': manager,
        'mission_name': 'suave',
    })


BT_SERVICES = [f'/{name}/change_mode' for name in (
    'f_maintain_motion', 'f_generate_search_path', 'f_follow_pipeline')]


def test_readiness_checks_listed_services_and_types():
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    try:
        runner.readiness_timeout = 0.0
        with patch.object(runner, 'get_service_names_and_types', return_value=[]):
            with pytest.raises(TimeoutError, match='f_maintain_motion'):
                runner._wait_for_readiness({'required_services': BT_SERVICES})
            runner._wait_for_readiness({'required_services': []})
        services = [(name, ['system_modes_msgs/srv/ChangeMode'])
                    for name in BT_SERVICES]
        with patch.object(runner, 'get_service_names_and_types',
                          return_value=services):
            runner._wait_for_readiness({'required_services': BT_SERVICES})
            with pytest.raises(TimeoutError, match='generate_recharge_path'):
                runner._wait_for_readiness({'required_services': BT_SERVICES + [
                    '/generate_recharge_path/change_mode']})
        with pytest.raises(ValueError, match='absolute'):
            runner._wait_for_readiness({'required_services': ['relative']})
        runner.terminate_flag = True
        with pytest.raises(RuntimeError, match='Termination'):
            runner._wait_for_readiness({'required_services': BT_SERVICES})
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_readiness_without_required_services_does_not_wait():
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    try:
        runner.readiness_timeout = 60.0
        with patch.object(runner, 'get_service_names_and_types',
                          side_effect=AssertionError('queried services')):
            start = time.monotonic()
            runner._wait_for_readiness(json.loads(_experiment('bt', 1)))
            assert time.monotonic() - start < 1.0
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_wait_until_returns_times_out_and_fails_early():
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    try:
        assert runner._wait_until(lambda: True, 0.0, 'Ready') < 1.0
        with pytest.raises(ReadinessTimeout, match='Stage not ready after 0.0s'):
            runner._wait_until(lambda: False, 0.0, 'Stage')
        runner._process_failure_queue = Queue()
        runner._process_failure_queue.put({
            'process_name': 'simulation launch service', 'pid': 7,
            'returncode': 1, 'cmd': ''})
        with pytest.raises(RuntimeError, match='exited with code 1'):
            runner._wait_until(lambda: False, 60.0, 'Stage')
        runner.terminate_flag = True
        with pytest.raises(RuntimeError, match='Termination'):
            runner._wait_until(lambda: False, 60.0, 'Stage')
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_port_listening_detects_listener_without_connecting():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server.bind(('127.0.0.1', 0))
        port = server.getsockname()[1]
        assert not port_listening(port)
        server.listen(1)
        assert port_listening(port)
    finally:
        server.close()
    assert not port_listening(port)


def test_udp_port_bound_detects_bound_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
        assert udp_port_bound(port)
    finally:
        sock.close()
    assert not udp_port_bound(port)


def test_check_no_stale_simulation_reports_gz_and_fdm_port():
    with patch('suave_runner.suave_runner.live_gz_sim_processes',
               return_value=[]), \
            patch('suave_runner.suave_runner.udp_port_bound',
                  return_value=False):
        check_no_stale_simulation()
    with patch('suave_runner.suave_runner.live_gz_sim_processes',
               return_value=[27297]), \
            patch('suave_runner.suave_runner.udp_port_bound',
                  return_value=True):
        with pytest.raises(StaleSimulationError,
                           match=r'gz sim PIDs \[27297\].*UDP port 9002'):
            check_no_stale_simulation()


def test_stale_simulation_aborts_campaign_before_attempt(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    runner.experiments = [json.loads(_experiment('bt', 2))]
    try:
        with patch('suave_runner.suave_runner.live_gz_sim_processes',
                   return_value=[27297]), \
                patch.object(runner, 'create_experiment_folder',
                             return_value=tmp_path), \
                patch.object(runner, 'generate_mission_config_files',
                             return_value=['config0.yaml', 'config1.yaml']), \
                patch.object(runner, 'launch_ardupilot') as launch_ardupilot:
            with pytest.raises(StaleSimulationError, match='27297'):
                runner.run_experiments()
        launch_ardupilot.assert_not_called()
        assert load_runs(tmp_path)['runs'] == {}
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_launch_ardupilot_waits_for_free_port_then_listener(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    try:
        assert '-N' in runner.ardupilot_cmd
        states = iter([True, False, False, True])
        with patch('suave_runner.suave_runner.port_listening',
                   side_effect=lambda port: next(states)), \
                patch('suave_runner.suave_runner.subprocess.Popen') as popen, \
                patch('suave_runner.suave_runner.time.sleep'):
            popen.return_value.poll.return_value = None
            runner.launch_ardupilot(tmp_path / 'ardupilot.log')
        popen.assert_called_once()
        assert next(states, 'done') == 'done'
        runner._ardupilot_log.close()
        runner.ardupilot_proc = None
        runner.ardupilot_ready_timeout = 0.0
        with patch('suave_runner.suave_runner.port_listening',
                   return_value=False), \
                patch('suave_runner.suave_runner.subprocess.Popen') as popen:
            popen.return_value.poll.return_value = None
            with pytest.raises(ReadinessTimeout, match='ArduPilot'):
                runner.launch_ardupilot(tmp_path / 'ardupilot.log')
        runner._ardupilot_log.close()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_next_attempt_ignores_previous_killed_ardupilot(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    try:
        killed = SimpleNamespace(pid=1, returncode=-9, poll=lambda: -9)
        runner.ardupilot_proc = killed
        with patch.object(runner, 'terminate_process'), \
                patch.object(runner, 'shutdown_all_launch_processes'), \
                patch('suave_runner.suave_runner.stop_sessions'):
            runner.shutdown_all_processes(tmp_path)
        assert runner.ardupilot_proc is None
        states = iter([False, True])
        with patch('suave_runner.suave_runner.port_listening',
                   side_effect=lambda port: next(states)), \
                patch('suave_runner.suave_runner.subprocess.Popen') as popen:
            popen.return_value.poll.return_value = None
            runner.launch_ardupilot(tmp_path / 'ardupilot.log')
        runner._ardupilot_log.close()
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_failed_readiness_continues_to_following_manager(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=(
        _minimal_runner_params() + [Parameter('record_bags', value=False)]))
    runner.experiments = [json.loads(_experiment('bt', 2)),
                          json.loads(_experiment('none', 1))]

    def launch(*args):
        runner._mission_done_event.set()

    try:
        with patch('suave_runner.suave_runner.check_no_stale_simulation'), \
                patch.object(runner, 'create_experiment_folder',
                             return_value=tmp_path), \
                patch.object(runner, 'generate_mission_config_files',
                             return_value=['config0.yaml', 'config1.yaml']), \
                patch.object(runner, 'launch_ardupilot'), \
                patch.object(runner, 'launch_suave_simulation'), \
                patch.object(runner, 'launch_experiment', side_effect=launch), \
                patch.object(runner, '_wait_for_readiness', side_effect=[
                    TimeoutError('Missing required services'),
                    ReadinessTimeout('Simulation not ready after 30.0s'),
                    None]), \
                patch.object(runner, 'shutdown_all_processes') as shutdown:
            runner.run_experiments()
        assert [call.args[0].parent.name for call in shutdown.call_args_list] == [
            'run_0_0', 'run_0_1', 'run_1_0']
        assert runner._active_log_dir is None
        runs = load_runs(tmp_path)['runs']
        assert runs['run_0_0']['status'] == 'failed'
        assert runs['run_0_0']['failure_type'] == 'readiness_missing_service'
        assert runs['run_0_1']['failure_type'] == 'readiness_timeout'
        assert runs['run_1_0']['status'] == 'completed'
        assert runs['run_1_0']['bag'] is None
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_shutdown_stops_sessions_after_launch_processes(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    calls = []
    try:
        with patch.object(runner, 'terminate_process',
                          side_effect=lambda *a: calls.append('ardupilot')), \
                patch.object(runner, 'shutdown_all_launch_processes',
                             side_effect=lambda: calls.append('launch')), \
                patch('suave_runner.suave_runner.stop_sessions',
                      side_effect=calls.append):
            runner.shutdown_all_processes(tmp_path / 'explicit')
            runner._active_log_dir = tmp_path / 'active'
            runner.shutdown_all_processes()
        assert calls == [
            'ardupilot', 'launch', tmp_path / 'explicit',
            'ardupilot', 'launch', tmp_path / 'active']
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_unconfirmed_shutdown_persists_reason_and_aborts(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=_minimal_runner_params())
    runner.experiments = [json.loads(_experiment('bt', 2))]

    def launch(*args):
        runner._mission_done_event.set()

    try:
        with patch('suave_runner.suave_runner.check_no_stale_simulation'), \
                patch.object(runner, 'create_experiment_folder',
                             return_value=tmp_path), \
                patch.object(runner, 'generate_mission_config_files',
                             return_value=['config0.yaml', 'config1.yaml']), \
                patch.object(runner, 'launch_ardupilot'), \
                patch.object(runner, 'launch_suave_simulation'), \
                patch.object(runner, 'launch_experiment', side_effect=launch), \
                patch.object(runner, '_wait_for_readiness'), \
                patch.object(runner, 'shutdown_all_processes',
                             side_effect=RuntimeError('still alive')):
            with pytest.raises(RuntimeError, match='still alive'):
                runner.run_experiments()
        runs = load_runs(tmp_path)['runs']
        assert runs['run_0_0']['status'] == 'interrupted'
        assert runs['run_0_0']['failure_type'] == 'shutdown_unconfirmed'
        assert runs['run_0_0']['bag'] == 'pending'
        assert 'run_0_1' not in runs
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_dry_campaign_resumes_from_runs_json(tmp_path):
    rclpy.init()
    runner = ExperimentRunnerNode(parameter_overrides=(
        _minimal_runner_params() + [Parameter('record_bags', value=False)]))
    runner.experiments = [json.loads(_experiment('bt', 3))]
    launched = []

    def launch(*args):
        launched.append(args[3])
        runner._mission_done_event.set()

    def run(readiness):
        with patch('suave_runner.suave_runner.check_no_stale_simulation'), \
                patch.object(runner, 'create_experiment_folder',
                             return_value=tmp_path), \
                patch.object(runner, 'generate_mission_config_files',
                             return_value=['c0.yaml', 'c1.yaml', 'c2.yaml']), \
                patch.object(runner, 'launch_ardupilot'), \
                patch.object(runner, 'launch_suave_simulation'), \
                patch.object(runner, 'launch_experiment', side_effect=launch), \
                patch.object(runner, '_wait_for_readiness',
                             side_effect=readiness), \
                patch.object(runner, 'shutdown_all_processes'):
            runner.run_experiments()

    try:
        run([None, TimeoutError('Missing required services'), None])
        first = load_runs(tmp_path)
        assert first['campaign']['random_seed'] == runner.random_seed
        assert [first['runs'][f'run_0_{i}']['status'] for i in range(3)] == [
            'completed', 'failed', 'completed']
        # A runner that died mid-run leaves its entry 'running'.
        runs = open_runs(tmp_path)
        start_attempt(runs, 'run_0_2', 'bt', 'suave', 'ros2 launch pkg file',
                      **attempt_details('run_0_2', 'bt', 'suave', False))
        write_runs(tmp_path, runs)
        runner.random_seed += 1
        launched.clear()
        run([None, None])
        assert launched == ['c1.yaml', 'c2.yaml']
        resumed = load_runs(tmp_path)
        assert resumed['campaign'] == first['campaign']
        assert resumed['runs']['run_0_0'] == first['runs']['run_0_0']
        run_0_1, run_0_2 = resumed['runs']['run_0_1'], resumed['runs']['run_0_2']
        assert run_0_1['status'] == 'completed'
        assert [a['failure_type'] for a in run_0_1['attempts']] == [
            'readiness_missing_service']
        assert run_0_2['status'] == 'completed'
        assert [a['status'] for a in run_0_2['attempts']] == [
            'completed', 'interrupted']
    finally:
        runner.destroy_node()
        rclpy.shutdown()


def test_create_experiment_folder():
    rclpy.init()
    try:
        params = [
            Parameter(
                'result_path',
                Parameter.Type.STRING,
                '/tmp/suave/results'),
            Parameter(
                'experiments',
                Parameter.Type.STRING_ARRAY,
                [_experiment('bt', 5)])
        ]
        runner = ExperimentRunnerNode(parameter_overrides=params)
        result_path = runner.create_experiment_folder()
        assert result_path.is_dir()
    finally:
        result_path.rmdir()
        path = Path('/tmp/suave')
        shutil.rmtree(path)
        rclpy.shutdown()


def test_random_seed_parameter_seeds_random_generator():
    rclpy.init()
    try:
        params = _minimal_runner_params() + [
            Parameter('random_seed', Parameter.Type.INTEGER, 42)
        ]
        with patch('suave_runner.suave_runner.random.seed') as seed:
            runner = ExperimentRunnerNode(parameter_overrides=params)

        assert runner.random_seed == 42
        seed.assert_called_once_with(42)
    finally:
        rclpy.shutdown()


def test_randomize_experiments_configuration():
    rclpy.init()
    try:
        params = [
            Parameter('gui', Parameter.Type.BOOL, True),
            Parameter('experiment_logging', Parameter.Type.BOOL, True),
            Parameter(
                'mission_config_pkg',
                Parameter.Type.STRING,
                'suave_missions'),
            Parameter(
                'mission_config_file',
                Parameter.Type.STRING,
                'config/mission_config.yaml'),
            Parameter(
                'water_visibility_sec_shift',
                Parameter.Type.DOUBLE,
                0.0),
            Parameter(
                'water_visibility_sec_shift_random_interval',
                Parameter.Type.DOUBLE_ARRAY,
                [-10.0, 10.0]),
            Parameter('random_interval', Parameter.Type.INTEGER, 5),
            Parameter('initial_pos_x', Parameter.Type.DOUBLE, -16.0),
            Parameter('initial_pos_y', Parameter.Type.DOUBLE, 2.0),
            Parameter(
                'initial_pos_x_random_interval',
                Parameter.Type.DOUBLE_ARRAY,
                [-1.0, 1.0]),
            Parameter(
                'initial_pos_y_random_interval',
                Parameter.Type.DOUBLE_ARRAY,
                [-1.0, 1.0]),
            Parameter('experiments', Parameter.Type.STRING_ARRAY, [
                _experiment('bt', 5),
                _experiment('metacontrol', 10),
                _experiment('random', 2),
                _experiment('none', 2),
            ])
        ]
        runner = ExperimentRunnerNode(parameter_overrides=params)
        runner.randomize_experiments_configuration()

        assert len(runner.initial_pos_x_array) == 10
        assert all(
            runner.initial_pos_x_array[i] == runner.initial_pos_x_array[0]
            for i in range(5))
        assert all(
            runner.initial_pos_x_array[i] == runner.initial_pos_x_array[5]
            for i in range(5, 10))
        assert runner.initial_pos_x_array[0] != runner.initial_pos_x_array[5]

        assert len(runner.initial_pos_y_array) == 10
        assert all(
            runner.initial_pos_y_array[i] == runner.initial_pos_y_array[0]
            for i in range(5))
        assert all(
            runner.initial_pos_y_array[i] == runner.initial_pos_y_array[5]
            for i in range(5, 10))
        assert runner.initial_pos_y_array[0] != runner.initial_pos_y_array[5]

        assert len(runner.initial_pos_z_array) == 10
        assert all(
            runner.initial_pos_z_array[i] == runner.initial_pos_z_array[0]
            for i in range(5))
        assert all(
            runner.initial_pos_z_array[i] == runner.initial_pos_z_array[5]
            for i in range(5, 10))

        assert len(runner.wv_sec_shift_array) == 10
        assert all(
            runner.wv_sec_shift_array[i] == runner.wv_sec_shift_array[0]
            for i in range(5))
        assert all(
            runner.wv_sec_shift_array[i] == runner.wv_sec_shift_array[5]
            for i in range(5, 10))
    finally:
        rclpy.shutdown()


def test_generate_mission_config_files():
    rclpy.init()
    try:
        params = [
            Parameter(
                'gui', Parameter.Type.BOOL, True),
            Parameter(
                'experiment_logging', Parameter.Type.BOOL, True),
            Parameter(
                'mission_config_pkg',
                Parameter.Type.STRING,
                'suave_missions'),
            Parameter(
                'mission_config_file',
                Parameter.Type.STRING,
                'config/mission_config.yaml'),
            Parameter(
                'water_visibility_sec_shift',
                Parameter.Type.DOUBLE,
                0.0),
            Parameter(
                'water_visibility_sec_shift_random_interval',
                Parameter.Type.DOUBLE_ARRAY,
                [-10.0, 10.0]),
            Parameter(
                'thruster_events',
                Parameter.Type.STRING_ARRAY,
                ['(1,failure,35)', '(3,failure,35)']),
            Parameter(
                'thruster_events_random_interval',
                Parameter.Type.DOUBLE_ARRAY,
                [-10.0, 10.0]),
            Parameter(
                'random_interval', Parameter.Type.INTEGER, 5),
            Parameter(
                'experiments',
                Parameter.Type.STRING_ARRAY,
                [
                    _experiment('bt', 5),
                    _experiment('metacontrol', 10),
                    _experiment('random', 2),
                    _experiment('none', 2),
                ])]
        runner = ExperimentRunnerNode(parameter_overrides=params)
        runner.randomize_experiments_configuration()
        result_path = Path('/tmp/suave/results/test_generate_mission_config/')
        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)
        config_files = runner.generate_mission_config_files(
            result_path)

        assert len(config_files) == 10

        wv_shift_array = []
        thruster_events_array = []
        for file in config_files:
            with open(file, 'r') as f:
                config = yaml.safe_load(f)
            observer_params = config[
                '/water_visibility_observer_node']['ros__parameters']
            wv_shift_array.append(
                float(observer_params['water_visibility_sec_shift']))
            thruster_params = config[
                '/thruster_monitor']['ros__parameters']
            thruster_events_array.append(
                thruster_params['thruster_events'])

        assert all(wv_shift_array[i] == wv_shift_array[0] for i in range(5))
        assert all(wv_shift_array[i] == wv_shift_array[5]
                   for i in range(5, 10))

        assert all(thruster_events_array[i][0][2] ==
                   thruster_events_array[0][0][2] for i in range(5))
        assert all(thruster_events_array[i][1][2] ==
                   thruster_events_array[0][1][2] for i in range(5))
        assert all(
            thruster_events_array[i][0][2] ==
            thruster_events_array[5][0][2]
            for i in range(5, 10))
        assert all(
            thruster_events_array[i][1][2] ==
            thruster_events_array[5][1][2]
            for i in range(5, 10))

    finally:
        path = Path('/tmp/suave')
        shutil.rmtree(path)
        rclpy.shutdown()


def test_mission_done_cb_sets_event():
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        assert not runner._mission_done_event.is_set()
        runner._mission_done_cb(Bool(data=True))
        assert runner._mission_done_event.is_set()
    finally:
        rclpy.shutdown()


def test_mission_done_cb_ignores_false():
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        runner._mission_done_cb(Bool(data=False))
        assert not runner._mission_done_event.is_set()
    finally:
        rclpy.shutdown()


def test_mission_failed_cb_records_reason_and_sets_event():
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        runner._mission_failed_cb(String(data='planner crashed'))

        assert runner._mission_failed_event.is_set()
        assert runner._mission_failure_reason == 'planner crashed'
    finally:
        rclpy.shutdown()


def test_launch_experiment_injects_water_current_when_enabled():
    rclpy.init()
    try:
        params = _minimal_runner_params() + [
            Parameter('enable_water_current', Parameter.Type.BOOL, True)
        ]
        runner = ExperimentRunnerNode(parameter_overrides=params)
        runner.start_launch_process = lambda *_args: (
            SimpleNamespace(), SimpleNamespace())

        with patch(
            'suave_runner.suave_runner.get_package_share_directory',
            return_value='/tmp'
        ), patch(
            'suave_runner.suave_runner.PythonLaunchDescriptionSource',
            side_effect=lambda path: path
        ), patch(
            'suave_runner.suave_runner.IncludeLaunchDescription'
        ) as include_launch:
            include_launch.side_effect = lambda source, launch_arguments: (
                SimpleNamespace(
                    source=source,
                    launch_arguments=dict(launch_arguments)))
            runner.launch_experiment(
                'ros2 launch suave_bringup mission.launch.py '
                'adaptation_manager:=bt',
                '/tmp/results',
                'bt_suave',
                '/tmp/mission_config.yaml',
                Path('/tmp/logs'))

        launch_args = dict(
            include_launch.call_args.kwargs['launch_arguments'])
        assert launch_args['enable_water_current'] == 'true'
    finally:
        rclpy.shutdown()


def test_launch_experiment_preserves_explicit_water_current_arg():
    rclpy.init()
    try:
        params = _minimal_runner_params() + [
            Parameter('enable_water_current', Parameter.Type.BOOL, True)
        ]
        runner = ExperimentRunnerNode(parameter_overrides=params)
        runner.start_launch_process = lambda *_args: (
            SimpleNamespace(), SimpleNamespace())

        with patch(
            'suave_runner.suave_runner.get_package_share_directory',
            return_value='/tmp'
        ), patch(
            'suave_runner.suave_runner.PythonLaunchDescriptionSource',
            side_effect=lambda path: path
        ), patch(
            'suave_runner.suave_runner.IncludeLaunchDescription'
        ) as include_launch:
            include_launch.side_effect = lambda source, launch_arguments: (
                SimpleNamespace(
                    source=source,
                    launch_arguments=dict(launch_arguments)))
            runner.launch_experiment(
                'ros2 launch suave_bringup mission.launch.py '
                'adaptation_manager:=bt enable_water_current:=false',
                '/tmp/results',
                'bt_suave',
                '/tmp/mission_config.yaml',
                Path('/tmp/logs'))

        launch_args = dict(
            include_launch.call_args.kwargs['launch_arguments'])
        assert launch_args['enable_water_current'] == 'false'
    finally:
        rclpy.shutdown()


def test_launch_experiment_passes_attempt_recording_path(tmp_path):
    """Verify runner-owned recording arguments are injected."""
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        runner.start_launch_process = lambda *_args: (
            SimpleNamespace(), SimpleNamespace())
        runner._active_attempt = ('run_0_0', start_attempt(
            open_runs(tmp_path), 'run_0_0', 'bt', 'suave', 'ros2 launch',
            **attempt_details('run_0_0', 'bt', 'suave', True)))

        with patch(
            'suave_runner.suave_runner.get_package_share_directory',
            return_value='/tmp'
        ), patch(
            'suave_runner.suave_runner.PythonLaunchDescriptionSource',
            side_effect=lambda path: path
        ), patch(
            'suave_runner.suave_runner.IncludeLaunchDescription'
        ) as include_launch:
            include_launch.side_effect = lambda source, launch_arguments: (
                SimpleNamespace(
                    source=source,
                    launch_arguments=dict(launch_arguments)))
            runner.launch_experiment(
                'ros2 launch suave_bringup mission.launch.py',
                str(tmp_path),
                'bt_suave',
                '/tmp/mission_config.yaml',
                Path('/tmp/logs'))

        launch_args = dict(
            include_launch.call_args.kwargs['launch_arguments'])
        assert launch_args['record_bags'] == 'true'
        assert launch_args['bag_output_path'].startswith(
            str(tmp_path / 'rosbags' / 'pending'))
    finally:
        rclpy.shutdown()


def test_launch_experiment_passes_recording_disabled(tmp_path):
    """Verify disabled recording is forwarded without a bag path."""
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        runner.start_launch_process = lambda *_args: (
            SimpleNamespace(), SimpleNamespace())
        runner._active_attempt = ('run_0_0', start_attempt(
            open_runs(tmp_path), 'run_0_0', 'bt', 'suave', 'ros2 launch',
            **attempt_details('run_0_0', 'bt', 'suave', False)))

        with patch(
            'suave_runner.suave_runner.get_package_share_directory',
            return_value='/tmp'
        ), patch(
            'suave_runner.suave_runner.PythonLaunchDescriptionSource',
            side_effect=lambda path: path
        ), patch(
            'suave_runner.suave_runner.IncludeLaunchDescription'
        ) as include_launch:
            include_launch.side_effect = lambda source, launch_arguments: (
                SimpleNamespace(
                    source=source,
                    launch_arguments=dict(launch_arguments)))
            runner.launch_experiment(
                'ros2 launch suave_bringup mission.launch.py',
                '/tmp/results',
                'bt_suave',
                '/tmp/mission_config.yaml',
                Path('/tmp/logs'))

        launch_args = dict(
            include_launch.call_args.kwargs['launch_arguments'])
        assert launch_args['record_bags'] == 'false'
        assert 'bag_output_path' not in launch_args
    finally:
        rclpy.shutdown()


def test_launch_experiment_passes_run_identity_env(tmp_path):
    """Pass the run index and attempt id to the experiment launch."""
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        calls = []
        runner.start_launch_process = lambda *args: (
            calls.append(args) or (SimpleNamespace(), SimpleNamespace()))
        entry = start_attempt(
            open_runs(tmp_path), 'run_1_4', 'bt', 'suave', 'ros2 launch',
            **attempt_details('run_1_4', 'bt', 'suave', False))
        runner._active_attempt = ('run_1_4', entry)

        with patch(
            'suave_runner.suave_runner.get_package_share_directory',
            return_value='/tmp'
        ), patch(
            'suave_runner.suave_runner.PythonLaunchDescriptionSource',
            side_effect=lambda path: path
        ), patch('suave_runner.suave_runner.IncludeLaunchDescription'):
            runner.launch_experiment(
                'ros2 launch suave_bringup mission.launch.py',
                str(tmp_path), 'bt_suave', '/tmp/mission_config.yaml',
                Path('/tmp/logs'))

        (_description, _log_dir, name, env), = calls
        assert name == 'experiment'
        assert env == {'SUAVE_RUN_IDX': '4',
                       'SUAVE_ATTEMPT_ID': entry['attempt_id']}
    finally:
        rclpy.shutdown()


def test_record_process_exit_queues_nonzero_exit():
    stop_event = SimpleNamespace(is_set=lambda: False)
    failure_queue = Queue()
    event = SimpleNamespace(
        returncode=-6,
        process_name='suave_bt_controller-12',
        pid=123,
        cmd=['suave_bt_controller', '--ros-args'],
    )

    ExperimentRunnerNode._record_process_exit(
        stop_event, failure_queue, event, None)

    assert failure_queue.get_nowait() == {
        'process_name': 'suave_bt_controller-12',
        'pid': 123,
        'returncode': -6,
        'cmd': 'suave_bt_controller --ros-args',
    }


def test_record_process_exit_ignores_zero_exit_and_shutdown():
    failure_queue = Queue()
    event = SimpleNamespace(
        returncode=0,
        process_name='owl_to_pddl-1',
        pid=123,
        cmd=['owl_to_pddl.py'],
    )
    running = SimpleNamespace(is_set=lambda: False)
    stopping = SimpleNamespace(is_set=lambda: True)

    ExperimentRunnerNode._record_process_exit(
        running, failure_queue, event, None)
    event.returncode = -15
    ExperimentRunnerNode._record_process_exit(
        stopping, failure_queue, event, None)

    try:
        failure_queue.get_nowait()
        assert False, 'No process failure should have been queued'
    except Empty:
        pass


def test_get_process_failure_reports_ardupilot_nonzero_exit():
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        runner.ardupilot_proc = SimpleNamespace(
            poll=lambda: -6,
            pid=456,
            returncode=-6,
        )

        assert runner._get_process_failure() == {
            'process_name': 'ArduPilot',
            'pid': 456,
            'returncode': -6,
            'cmd': ' '.join(runner.ardupilot_cmd),
        }
    finally:
        rclpy.shutdown()


def test_handle_termination_sets_done_event():
    rclpy.init()
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        assert not runner._mission_done_event.is_set()
        runner.handle_termination()
        assert runner._mission_done_event.is_set()
    finally:
        rclpy.shutdown()
