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

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction

from launch_ros.actions import Node


def _load_launch_module():
    launch_path = (
        Path(__file__).parents[1] / 'launch' / 'suave_base.launch.py')
    spec = importlib.util.spec_from_file_location(
        'suave_base_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_launch_description():
    return _load_launch_module().generate_launch_description()


def _bag_recorder(launch_configurations):
    entities = _load_launch_description().entities
    recorders = [e for e in entities if isinstance(e, OpaqueFunction)]
    assert len(recorders) == 1
    context = LaunchContext()
    context.launch_configurations.update(launch_configurations)
    return recorders[0], context


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


def test_launch_includes_requirement_monitor():
    """Verify the requirement monitor is launched once."""
    entities = _load_launch_description().entities
    monitors = [
        e for e in entities
        if isinstance(e, Node) and e._Node__package == 'suave_requirements']
    assert len(monitors) == 1


def test_record_bags_defaults_to_true():
    """Verify bag recording is enabled by default."""
    entities = _load_launch_description().entities
    record_bags = [
        e for e in entities
        if isinstance(e, DeclareLaunchArgument) and e.name == 'record_bags']
    assert len(record_bags) == 1
    assert record_bags[0].default_value[0].text == 'true'


def test_record_bags_false_skips_recorder():
    """Verify no recorder process is started when recording is disabled."""
    recorder, context = _bag_recorder({'record_bags': 'false'})
    assert not recorder.condition.evaluate(context)


def test_recorder_command(tmp_path, monkeypatch):
    """Verify the recorder uses MCAP, explicit topics, and result_path."""
    monkeypatch.setenv('HOME', str(tmp_path))
    recorder, context = _bag_recorder({
        'record_bags': 'true',
        'result_path': '~/results',
        'result_filename': 'bt_time_constrained_mission',
    })
    assert recorder.condition.evaluate(context)
    actions = recorder.execute(context)
    assert len(actions) == 1 and isinstance(actions[0], ExecuteProcess)
    cmd = [''.join(s.perform(context) for s in part)
           for part in actions[0].cmd]
    assert cmd[:3] == ['ros2', 'bag', 'record']
    assert cmd[cmd.index('-s') + 1] == 'mcap'
    assert cmd[cmd.index('--storage-preset-profile') + 1] == 'fastwrite'
    assert '-a' not in cmd and '--all' not in cmd
    assert not any(arg.startswith('--compression') for arg in cmd)

    output = Path(cmd[cmd.index('-o') + 1])
    assert output.parent == tmp_path / 'results' / 'rosbags'
    assert output.parent.is_dir() and not output.exists()
    assert output.name.startswith('bt_time_constrained_mission_')

    topics = _load_launch_module().RECORDED_TOPICS
    assert cmd[-len(topics):] == topics
    assert len(topics) == len(set(topics))
    assert all(topic.startswith('/') for topic in topics)
    assert '/requirements/search_footprint/fulfillment' in topics


def test_recorder_name_without_result_filename(tmp_path):
    """Verify the bag name falls back to a suave prefix."""
    recorder, context = _bag_recorder({
        'record_bags': 'true',
        'result_path': str(tmp_path),
        'result_filename': '',
    })
    cmd = [''.join(s.perform(context) for s in part)
           for part in recorder.execute(context)[0].cmd]
    assert Path(cmd[cmd.index('-o') + 1]).name.startswith('suave_')
