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
import os
from pathlib import Path
from queue import Empty
from queue import Queue
import shutil
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import String

from suave_runner.suave_runner import ExperimentRunnerNode
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


def test_record_process_exit_queues_nonzero_exit():
    stop_event = SimpleNamespace(is_set=lambda: False)
    failure_queue = Queue()
    event = SimpleNamespace(
        returncode=-6,
        process_name='suave_planta_controller-12',
        pid=123,
        cmd=['suave_planta_controller', '--ros-args'],
    )

    ExperimentRunnerNode._record_process_exit(
        stop_event, failure_queue, event, None)

    assert failure_queue.get_nowait() == {
        'process_name': 'suave_planta_controller-12',
        'pid': 123,
        'returncode': -6,
        'cmd': 'suave_planta_controller --ros-args',
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


def _write_node_log(experiment_dir, name, text, mtime=None):
    experiment_dir.mkdir(parents=True, exist_ok=True)
    log_path = experiment_dir / name
    log_path.write_text(text)
    if mtime is not None:
        os.utime(log_path, (mtime, mtime))
    return log_path


def test_detect_broken_run_flags_configured_but_not_activated():
    rclpy.init()
    workdir = Path(tempfile.mkdtemp())
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        _write_node_log(
            workdir / 'experiment', 'python3_10_1000.log',
            '[f_generate_search_path_node]: on_configure() is called.\n')
        assert runner.detect_broken_run(workdir)
    finally:
        rclpy.shutdown()
        shutil.rmtree(workdir, ignore_errors=True)


def test_detect_broken_run_passes_when_node_activated():
    rclpy.init()
    workdir = Path(tempfile.mkdtemp())
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        _write_node_log(
            workdir / 'experiment', 'python3_10_1000.log',
            '[f_generate_search_path_node]: on_configure() is called.\n'
            '[f_generate_search_path_node]: on_activate() is called.\n')
        assert runner.detect_broken_run(workdir) == ''
    finally:
        rclpy.shutdown()
        shutil.rmtree(workdir, ignore_errors=True)


def test_detect_broken_run_uses_latest_attempt():
    rclpy.init()
    workdir = Path(tempfile.mkdtemp())
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        experiment_dir = workdir / 'experiment'
        _write_node_log(
            experiment_dir, 'python3_10_1000.log',
            '[f_generate_search_path_node]: on_configure() is called.\n',
            mtime=1000)
        _write_node_log(
            experiment_dir, 'python3_20_2000.log',
            '[f_generate_search_path_node]: on_configure() is called.\n'
            '[f_generate_search_path_node]: on_activate() is called.\n',
            mtime=2000)
        assert runner.detect_broken_run(workdir) == ''
    finally:
        rclpy.shutdown()
        shutil.rmtree(workdir, ignore_errors=True)


def test_record_broken_run_quarantines_last_results_row():
    rclpy.init()
    workdir = Path(tempfile.mkdtemp())
    try:
        runner = ExperimentRunnerNode(
            parameter_overrides=_minimal_runner_params())
        results_csv = workdir / 'bt_suave.csv'
        results_csv.write_text(
            'mission name,pipeline found\nrow-a,True\nrow-b,False\n')

        runner.record_broken_run(
            workdir, 2, 6, 'bt', 'bt_suave', 'reason text')

        remaining = results_csv.read_text()
        assert 'row-a' in remaining
        assert 'row-b' not in remaining
        assert 'row-b,False' in (
            workdir / 'bt_suave_rejected.csv').read_text()
        manifest = (workdir / 'rejected_runs.jsonl').read_text()
        assert '"run": "run_2_6"' in manifest
    finally:
        rclpy.shutdown()
        shutil.rmtree(workdir, ignore_errors=True)
