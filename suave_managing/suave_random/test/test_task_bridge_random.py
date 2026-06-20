# Copyright 2026 KAS Lab
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

"""Tests for the random task bridge configuration."""

import importlib.util
from pathlib import Path

from launch_ros.actions import Node

import pytest
import rclpy

from suave_random.task_bridge_random import TaskBridgeRandom
import yaml


@pytest.fixture(scope='module', autouse=True)
def rclpy_runtime():
    """Initialize rclpy with an adapt_period override."""
    if not rclpy.ok():
        rclpy.init(args=['--ros-args', '-p', 'adapt_period:=7'])
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_task_bridge_random_reads_adapt_period_parameter():
    """Verify the random bridge reads the parameter used by mission YAML."""
    node = TaskBridgeRandom()
    try:
        assert node.get_parameter('adapt_period').value == 7
        assert node.adaptation_period == 7
    finally:
        node.destroy_node()


def test_random_launch_names_task_bridge_for_yaml_scope():
    """Verify launch node name matches the /task_bridge YAML scope."""
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'suave_random.launch.py')
    spec = importlib.util.spec_from_file_location(
        'suave_random_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    launch_description = module.generate_launch_description()
    task_bridge_nodes = [
        entity
        for entity in launch_description.entities
        if isinstance(entity, Node) and
        entity._Node__package == 'suave_random' and
        entity._Node__node_executable == 'task_bridge_random'
    ]

    assert len(task_bridge_nodes) == 1
    assert task_bridge_nodes[0]._Node__node_name == 'task_bridge'


@pytest.mark.parametrize(
    'config_path',
    sorted((Path(__file__).parents[3] / 'suave_missions' / 'config').glob(
        '*mission_config.yaml'))
)
def test_mission_configs_use_declared_random_adaptation_parameter(
        config_path):
    """Verify mission YAML files use the declared random bridge parameter."""
    config = yaml.safe_load(config_path.read_text())
    task_bridge_params = config['/task_bridge']['ros__parameters']

    assert 'adapt_period' in task_bridge_params
    assert 'adaptation_period' not in task_bridge_params
