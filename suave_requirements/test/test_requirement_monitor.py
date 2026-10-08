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

"""Tests for the requirement_monitor node callbacks."""

import math

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import pytest

import rclpy
from rclpy.parameter import Parameter

from suave_msgs.msg import ReactionTime

from suave_requirements.requirement_monitor import RequirementMonitor


@pytest.fixture
def node():
    """Create a RequirementMonitor in an isolated context."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    monitor = RequirementMonitor(
        'test_requirement_monitor',
        context=context,
        parameter_overrides=[
            Parameter('reaction_time_window_size', value=2)])
    try:
        yield monitor
    finally:
        monitor.destroy_node()
        rclpy.shutdown(context=context)


def _diagnostics(key, value):
    status = DiagnosticStatus(message='QA status')
    status.values.append(KeyValue(key=key, value=value))
    return DiagnosticArray(status=[status])


def _reaction_time(adaptation_type, latest, count,
                   status=ReactionTime.STATUS_VALID):
    return ReactionTime(
        adaptation_type=adaptation_type, status=status,
        latest=latest, mean=latest, count=count)


def test_initial_fulfillment(node):
    """Every fulfillment is unknown before any input."""
    assert node.fulfillment['thruster_availability'] is None
    assert node.fulfillment['search_footprint'] is None
    assert node.fulfillment['adaptation_reaction_time'] is None
    assert node.fulfillment['adaptation_reaction_time/battery'] is None


def test_thruster_availability_from_diagnostics(node):
    """Operational thruster count updates thruster availability."""
    node.diagnostics_cb(_diagnostics('operational_thrusters', '6'))
    assert node.fulfillment['thruster_availability'] == 1.0
    node.diagnostics_cb(_diagnostics('operational_thrusters', '5'))
    assert node.fulfillment['thruster_availability'] == 0.0


def test_search_footprint_from_diagnostics(node):
    """Coverage area updates the search footprint fulfillment."""
    node.diagnostics_cb(_diagnostics('coverage_area', '6.0'))
    assert node.fulfillment['search_footprint'] == pytest.approx(0.5)


def test_unrelated_diagnostics_are_ignored(node):
    """Other keys and non-numeric values leave fulfillment unchanged."""
    node.diagnostics_cb(_diagnostics('water_visibility', '2.0'))
    node.diagnostics_cb(_diagnostics('coverage_area', 'nan?'))
    assert node.fulfillment['search_footprint'] is None
    assert node.fulfillment['thruster_availability'] is None


def test_reaction_time_per_type_and_combined(node):
    """New reactions update per-type and moving-average fulfillment."""
    node.reaction_time_cb(_reaction_time(
        ReactionTime.ADAPTATION_THRUSTER, 5.0, 1))
    assert node.fulfillment['adaptation_reaction_time/thruster'] == \
        pytest.approx(math.exp(-1.0))
    assert node.fulfillment['adaptation_reaction_time/battery'] is None
    assert node.fulfillment['adaptation_reaction_time'] == \
        pytest.approx(math.exp(-1.0))

    node.reaction_time_cb(_reaction_time(
        ReactionTime.ADAPTATION_BATTERY, 0.0, 1))
    assert node.fulfillment['adaptation_reaction_time'] == \
        pytest.approx((math.exp(-1.0) + 1.0) / 2)

    node.reaction_time_cb(_reaction_time(
        ReactionTime.ADAPTATION_BATTERY, 10.0, 2))
    # Window of 2 keeps only the two battery reactions.
    assert node.fulfillment['adaptation_reaction_time'] == \
        pytest.approx((1.0 + math.exp(-2.0)) / 2)


def test_invalid_reaction_time_is_ignored(node):
    """Invalid reaction times leave fulfillment unknown."""
    node.reaction_time_cb(_reaction_time(
        ReactionTime.ADAPTATION_WATER_VISIBILITY, 0.0, 0,
        status=ReactionTime.STATUS_INVALID))
    assert node.fulfillment[
        'adaptation_reaction_time/water_visibility'] is None
    assert node.fulfillment['adaptation_reaction_time'] is None
    assert len(node.reaction_time_average._samples) == 0


def test_unknown_adaptation_type_is_ignored(node):
    """Reaction times of unknown adaptation types are not used."""
    node.reaction_time_cb(_reaction_time('unknown', 50.0, 1))
    assert 'adaptation_reaction_time/unknown' not in node.fulfillment
    assert node.fulfillment['adaptation_reaction_time'] is None
    assert len(node.reaction_time_average._samples) == 0


def test_repeated_reaction_message_is_not_counted_twice(node):
    """A republished message with the same count adds no samples."""
    msg = _reaction_time(ReactionTime.ADAPTATION_WATER_VISIBILITY, 5.0, 1)
    node.reaction_time_cb(msg)
    node.reaction_time_cb(msg)
    assert len(node.reaction_time_average._samples) == 1


def test_thruster_availability_uses_fuzzy_parameters():
    """Thruster foot and shoulder come from ROS parameters."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    monitor = RequirementMonitor(
        'test_requirement_monitor_thruster_params',
        context=context,
        parameter_overrides=[
            Parameter('thruster_availability_foot', value=2.0),
            Parameter('thruster_availability_shoulder', value=0.0)])
    try:
        monitor.diagnostics_cb(_diagnostics('operational_thrusters', '5'))
        assert monitor.fulfillment['thruster_availability'] == \
            pytest.approx(0.5)
    finally:
        monitor.destroy_node()
        rclpy.shutdown(context=context)
