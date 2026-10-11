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

"""Tests for MissionMetrics done-signal behaviour."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import rclpy
from rclpy.parameter import Parameter

from std_msgs.msg import Bool

from suave_metrics.mission_metrics import MissionMetrics


@pytest.fixture
def rclpy_context():
    """Create an isolated rclpy context."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


def _mission_metrics(context, parameter_overrides):
    return MissionMetrics(
        'test_mission_metrics_done',
        context=context,
        parameter_overrides=parameter_overrides)


def test_progress_callbacks_keep_handles_and_first_values(rclpy_context):
    node = _mission_metrics(rclpy_context, [])
    try:
        with patch.object(node, 'destroy_subscription') as destroy:
            node.status_cb(SimpleNamespace(mode='GUIDED'))
            start = node.mission_start_time
            node.status_cb(SimpleNamespace(mode='GUIDED'))
            assert node.mission_start_time == start
            node.pipeline_detected_cb(Bool(data=True))
            detected = node.pipeline_detected_time
            node.pipeline_detected_cb(Bool(data=True))
            assert node.pipeline_detected_time == detected
            node.gazebo_pos_cb(SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0)))
            node.gazebo_pos_cb(SimpleNamespace(
                position=SimpleNamespace(x=3.0, y=4.0)))
            assert (node.initial_x, node.initial_y) == (1.0, 2.0)
            node.results_saved = True
            node.distance_inspected_cb(SimpleNamespace(data=12.0))
            assert node.distance_inspected == 0.0
            destroy.assert_not_called()
    finally:
        node.destroy_node()


def test_concurrent_save_is_once(rclpy_context):
    node = _mission_metrics(rclpy_context, [])
    try:
        node.mission_start_time = node.get_clock().now()
        with patch.object(node, '_write_mission_results') as write:
            with ThreadPoolExecutor(max_workers=4) as executor:
                list(executor.map(lambda _: node.save_mission_results(), range(20)))
            node.pipeline_inspected_cb(Bool(data=True))
            node.save_mission_results_cb(None, None)
            write.assert_called_once()
            assert node.results_saved
    finally:
        node.destroy_node()


def test_save_before_guided_does_not_signal_completion(rclpy_context):
    node = _mission_metrics(rclpy_context, [])
    try:
        with patch.object(node.mission_done_pub, 'publish') as publish:
            node.save_mission_results()
            publish.assert_not_called()
            assert not node.results_saved
    finally:
        node.destroy_node()


def test_main_handles_external_shutdown():
    from suave_metrics import mission_metrics

    with patch.object(mission_metrics.rclpy, 'init'), \
            patch.object(mission_metrics, 'MissionMetrics') as factory, \
            patch.object(mission_metrics.rclpy.executors,
                         'MultiThreadedExecutor') as executor, \
            patch.object(mission_metrics.rclpy, 'try_shutdown') as shutdown:
        executor.return_value.spin.side_effect = (
            rclpy.executors.ExternalShutdownException())
        mission_metrics.main()
        executor.return_value.shutdown.assert_called_once()
        factory.return_value.destroy_node.assert_called_once()
        shutdown.assert_called_once()


def test_save_mission_results_creates_done_file(rclpy_context, tmp_path):
    """save_mission_results() creates /tmp/mission.done for shell-script compat."""
    done_path = Path('/tmp/mission.done')
    done_path.unlink(missing_ok=True)
    node = _mission_metrics(
        rclpy_context,
        [Parameter('result_path', value=str(tmp_path))])
    try:
        node.mission_start_time = node.get_clock().now()
        with patch.object(node, 'save_metrics'):
            node.save_mission_results()
        assert done_path.exists()
    finally:
        done_path.unlink(missing_ok=True)
        node.destroy_node()


def test_save_mission_results_publishes_done_topic(rclpy_context, tmp_path):
    """save_mission_results() publishes Bool(True) on mission_metrics/done."""
    node = _mission_metrics(
        rclpy_context,
        [Parameter('result_path', value=str(tmp_path))])
    try:
        node.mission_start_time = node.get_clock().now()
        with patch.object(node, 'save_metrics'), \
                patch.object(node.mission_done_pub, 'publish') as mock_pub:
            node.save_mission_results()
        mock_pub.assert_called_once()
        published_msg = mock_pub.call_args[0][0]
        assert isinstance(published_msg, Bool)
        assert published_msg.data is True
    finally:
        node.destroy_node()
