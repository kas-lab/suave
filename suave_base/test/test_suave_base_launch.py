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

"""Tests for the SUAVE base launch description."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node


def _load_launch_description():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'suave_base.launch.py')
    spec = importlib.util.spec_from_file_location(
        'suave_base_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_has_no_duplicate_arguments():
    """Verify no argument is declared twice."""
    entities = _load_launch_description().entities
    arguments = [
        e.name for e in entities if isinstance(e, DeclareLaunchArgument)]
    assert len(arguments) == len(set(arguments))


def test_launch_includes_managed_system():
    """Verify the managed system is included exactly once."""
    entities = _load_launch_description().entities
    includes = [e for e in entities if isinstance(e, IncludeLaunchDescription)]
    assert len(includes) == 1


def test_launch_includes_metrics_nodes():
    """Verify two conditioned metrics node variants are present."""
    entities = _load_launch_description().entities
    metrics_nodes = [
        e for e in entities
        if isinstance(e, Node) and e._Node__package == 'suave_metrics']
    assert len(metrics_nodes) == 2


def test_launch_has_no_mission_node():
    """Verify no mission node is launched from the base."""
    entities = _load_launch_description().entities
    mission_nodes = [
        e for e in entities
        if isinstance(e, Node) and e._Node__package == 'suave_missions']
    assert len(mission_nodes) == 0
