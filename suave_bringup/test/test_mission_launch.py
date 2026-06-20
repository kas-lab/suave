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

"""Tests for the top-level mission launch description."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node


def _load_launch_description():
    launch_path = Path(__file__).parents[1] / 'launch' / 'mission.launch.py'
    spec = importlib.util.spec_from_file_location('mission_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_composes_common_nodes_and_each_manager():
    """Verify common nodes and all manager launch choices are present."""
    entities = _load_launch_description().entities
    arguments = [
        entity.name for entity in entities
        if isinstance(entity, DeclareLaunchArgument)]
    includes = [
        entity for entity in entities
        if isinstance(entity, IncludeLaunchDescription)]
    nodes = [entity for entity in entities if isinstance(entity, Node)]

    assert len(arguments) == len(set(arguments))
    assert len(includes) == 5
    assert sum(node._Node__package == 'suave_missions' for node in nodes) == 1
    assert sum(node._Node__package == 'suave_metrics' for node in nodes) == 2
