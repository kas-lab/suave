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

"""Tests for inspection mission subscription callbacks."""

from unittest.mock import patch

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool

from suave_missions.time_constrained_mission import MissionTimeConstrained


@pytest.fixture
def mission():
    """Create a mission node without spinning it."""
    rclpy.init()
    node = MissionTimeConstrained()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _battery(level):
    return DiagnosticArray(status=[DiagnosticStatus(
        message='QA status',
        values=[KeyValue(key='battery_level', value=str(level))])])


def test_pipeline_detection_latches_without_destroying_subscription(mission):
    """Keep the first detection even if a later message reports False."""
    with patch.object(mission, 'destroy_subscription') as destroy:
        mission.pipeline_detected_cb(Bool(data=False))
        assert mission.pipeline_detected is False
        mission.pipeline_detected_cb(Bool(data=True))
        detected = mission.pipeline_detected_time
        mission.pipeline_detected_cb(Bool(data=False))
        mission.pipeline_detected_cb(Bool(data=True))
        assert mission.pipeline_detected is True
        assert mission.pipeline_detected_time == detected
        destroy.assert_not_called()


def test_low_battery_aborts_once_without_destroying_subscription(mission):
    """Save results once on low battery and keep the subscription."""
    mission.set_parameters([
        Parameter('battery_constraint', value=True),
        Parameter('battery_constraint_value', value=0.5)])
    with patch.object(mission, 'destroy_subscription') as destroy, \
            patch.object(mission, 'call_service') as call_service:
        mission.battery_level_cb(_battery(0.9))
        assert mission.abort_mission is False
        mission.battery_level_cb(_battery(0.1))
        mission.battery_level_cb(_battery(0.1))
        assert mission.abort_mission is True
        call_service.assert_called_once()
        destroy.assert_not_called()
