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

from pathlib import Path
import shutil
from unittest.mock import patch

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from suave_runner.suave_runner import ExperimentRunnerNode
import yaml


def _minimal_runner_params():
    return [
        Parameter('experiments', Parameter.Type.STRING_ARRAY, [
            '{"experiment_launch": "ros2 launch suave_bt suave_bt.launch.py",'
            ' "num_runs": 1, "adaptation_manager": "bt",'
            ' "mission_name": "suave"}'
        ])
    ]


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
                ["""{
                  "experiment_launch":
                    "ros2 launch suave_bt suave_bt.launch.py",
                  "num_runs": 5,
                  "adaptation_manager": "bt",
                  "mission_name": "suave"
                }"""])
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
                """{
                  "experiment_launch":
                    "ros2 launch suave_bt suave_bt.launch.py",
                  "num_runs": 5,
                  "adaptation_manager": "bt",
                  "mission_name": "suave"
                }""",
                """{
                  "experiment_launch":
                  "ros2 launch suave_metacontrol suave_metacontrol.launch.py",
                  "num_runs": 10,
                  "adaptation_manager": "metacontrol",
                  "mission_name": "suave"
                }""",
                """{
                  "experiment_launch":
                    "ros2 launch suave_random suave_random.launch.py",
                  "num_runs": 2,
                  "adaptation_manager": "random",
                  "mission_name": "suave"
                }""",
                """{
                  "experiment_launch":
                    "ros2 launch suave_none suave_none.launch.py",
                  "num_runs": 2,
                  "adaptation_manager": "none",
                  "mission_name": "suave"
                }"""
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
                ["""{
                  "experiment_launch":
                    "ros2 launch suave_bt suave_bt.launch.py",
                  "num_runs": 5,
                  "adaptation_manager": "bt",
                  "mission_name": "suave"
                }""", """{
                  "experiment_launch":
                  "ros2 launch suave_metacontrol suave_metacontrol.launch.py",
                  "num_runs": 10,
                  "adaptation_manager": "metacontrol",
                  "mission_name": "suave"
                }""", """{
                  "experiment_launch":
                    "ros2 launch suave_random suave_random.launch.py",
                  "num_runs": 2,
                  "adaptation_manager": "random",
                  "mission_name": "suave"
                }""", """{
                  "experiment_launch":
                    "ros2 launch suave_none suave_none.launch.py",
                  "num_runs": 2,
                  "adaptation_manager": "none",
                  "mission_name": "suave"
                }"""])]
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
