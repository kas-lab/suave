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

"""Tests for the ReactionTime messages published by MissionMetrics."""

from unittest.mock import patch

import pytest

import rclpy

from suave_metrics.mission_metrics import build_reaction_time_msg
from suave_metrics.mission_metrics import MissionMetrics

from suave_msgs.msg import ReactionTime


def test_reaction_time_msg_without_samples_is_invalid():
    """An adaptation without reactions is invalid with zeroed values."""
    msg = build_reaction_time_msg(ReactionTime.ADAPTATION_BATTERY, [])
    assert msg.adaptation_type == 'battery'
    assert msg.status == ReactionTime.STATUS_INVALID
    assert msg.latest == 0.0
    assert msg.mean == 0.0
    assert msg.count == 0


def test_reaction_time_msg_latest_and_mean():
    """An adaptation with reactions reports its last value, mean, and count."""
    msg = build_reaction_time_msg(
        ReactionTime.ADAPTATION_THRUSTER, [3.0, 6.0, 9.0])
    assert msg.adaptation_type == 'thruster'
    assert msg.status == ReactionTime.STATUS_VALID
    assert msg.latest == pytest.approx(9.0)
    assert msg.mean == pytest.approx(6.0)
    assert msg.count == 3


def test_mission_metrics_announces_invalid_reaction_times():
    """Startup publishes one invalid message per adaptation type."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        with patch.object(
                MissionMetrics, 'publish_reaction_time',
                autospec=True) as publish:
            node = MissionMetrics(
                'test_reaction_time_announce', context=context)
        announced = [
            build_reaction_time_msg(*call.args[1:])
            for call in publish.call_args_list]
        assert [(m.adaptation_type, m.status) for m in announced] == [
            ('thruster', ReactionTime.STATUS_INVALID),
            ('water_visibility', ReactionTime.STATUS_INVALID),
            ('battery', ReactionTime.STATUS_INVALID),
        ]
        node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
