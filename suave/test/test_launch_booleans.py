# Copyright 2026 KAS-lab
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

"""Evaluate real launch conditions without starting managed processes."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import pytest


def _conditions():
    path = Path(__file__).parents[1] / 'launch' / 'suave.launch.py'
    spec = importlib.util.spec_from_file_location('suave_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description().entities
    bridge = next(a for a in actions if isinstance(a, Node) and
                  a.node_executable == 'task_bridge_none')
    modes = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    return bridge.condition, modes.condition


@pytest.mark.parametrize('value,enabled', [
    ('true', True), ('True', True), ('1', True), (' TRUE ', True),
    ('false', False), ('False', False), ('0', False),
])
def test_boolean_conditions(value, enabled):
    context = LaunchContext()
    context.launch_configurations.update(task_bridge=value, system_modes=value)
    for condition in _conditions():
        assert condition.evaluate(context) is enabled


def test_invalid_boolean_fails_explicitly():
    context = LaunchContext()
    context.launch_configurations.update(task_bridge='invalid',
                                         system_modes='invalid')
    for condition in _conditions():
        with pytest.raises(Exception, match='invalid'):
            condition.evaluate(context)
