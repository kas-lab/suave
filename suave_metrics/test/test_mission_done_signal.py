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

from pathlib import Path
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
