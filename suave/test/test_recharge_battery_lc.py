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

"""Tests for the recharge battery lifecycle node action server."""

import pytest
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from action_test_utils import send_goal_and_wait
from action_test_utils import spin_nodes_in_executors
from action_test_utils import wait_for_condition
from suave.action_server_utils import lifecycle_state_is_active
from suave import recharge_battery_lc
from suave.recharge_battery_lc import RechargeBattery
from suave_msgs.action import RechargeBattery as RechargeBatteryAction


@pytest.fixture(scope='session', autouse=True)
def rclpy_runtime():
    """Initialize rclpy for the test module."""
    if not rclpy.ok():
        rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@pytest.fixture
def test_node():
    """Create a helper ROS node."""
    node = Node('test_recharge_battery_helper')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def recharge_node():
    """Create the recharge battery lifecycle node under test."""
    node = RechargeBattery('test_recharge_battery_lc')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def executor(test_node, recharge_node):
    """Spin helper and recharge nodes in separate executors."""
    with spin_nodes_in_executors(test_node, recharge_node) as executors:
        yield executors[-1]


def test_recharge_battery_action_rejected_when_inactive(
        executor, test_node, recharge_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(
        test_node, RechargeBatteryAction, 'recharge_battery')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, RechargeBatteryAction.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    client.destroy()


def test_recharge_battery_action_rejected_when_use_action_server_false(
        executor, monkeypatch, test_node, recharge_node):
    """Verify active goals are rejected when use_action_server=False."""
    monkeypatch.setattr(recharge_node, '_start_legacy_recharge', lambda: None)
    recharge_node.trigger_activate()
    client = ActionClient(
        test_node, RechargeBatteryAction, 'recharge_battery')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, RechargeBatteryAction.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    recharge_node.trigger_deactivate()
    client.destroy()


def test_recharge_battery_use_action_server_default_is_false(recharge_node):
    """Verify use_action_server defaults to False."""
    assert recharge_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False


def test_recharge_retry_rate_default_is_two_hertz(recharge_node):
    """Verify the shared recharge runner defaults to two attempts per second."""
    assert recharge_node.get_parameter(
        'recharge_retry_rate').get_parameter_value().double_value == 2.0


def test_get_recharge_station_pose_uses_configured_position(recharge_node):
    """Verify station pose coordinates come from the ROS parameter."""
    recharge_node.set_parameters([
        rclpy.parameter.Parameter(
            'recharge_station_gz_pos', value=[1.5, -2.5, -8.0])])

    pose = recharge_node._get_recharge_station_pose()

    assert pose.position.x == 1.5
    assert pose.position.y == -2.5
    assert pose.position.z == -8.0


def test_try_recharge_once_in_progress_when_setpoint_unavailable(
        monkeypatch, recharge_node):
    """Verify a missing setpoint leaves recharge in progress."""
    monkeypatch.setattr(
        recharge_node.ardusub,
        'setpoint_position_gz',
        lambda pose, fixed_altitude=True: None)

    assert recharge_node._try_recharge_once() is None


def test_try_recharge_once_in_progress_until_setpoint_reached(
        monkeypatch, recharge_node):
    """Verify movement toward an unreached setpoint remains in progress."""
    setpoint = object()
    monkeypatch.setattr(
        recharge_node.ardusub,
        'setpoint_position_gz',
        lambda pose, fixed_altitude=True: setpoint)
    monkeypatch.setattr(
        recharge_node.ardusub,
        'check_setpoint_reached_xy',
        lambda candidate, threshold: False)

    assert recharge_node._try_recharge_once() is None


def test_try_recharge_once_returns_true_when_service_succeeds(
        monkeypatch, recharge_node):
    """Verify a successful recharge service response completes recharge."""
    setpoint = object()
    response = Trigger.Response(success=True)
    monkeypatch.setattr(
        recharge_node.ardusub,
        'setpoint_position_gz',
        lambda pose, fixed_altitude=True: setpoint)
    monkeypatch.setattr(
        recharge_node.ardusub,
        'check_setpoint_reached_xy',
        lambda candidate, threshold: True)
    monkeypatch.setattr(
        recharge_battery_lc,
        'call_service_with_timeout',
        lambda node, client, request: response)

    assert recharge_node._try_recharge_once() is True


@pytest.mark.parametrize('response', [None, Trigger.Response(success=False)])
def test_try_recharge_once_returns_false_when_service_fails(
        monkeypatch, recharge_node, response):
    """Verify unavailable or rejected recharge service calls fail recharge."""
    setpoint = object()
    monkeypatch.setattr(
        recharge_node.ardusub,
        'setpoint_position_gz',
        lambda pose, fixed_altitude=True: setpoint)
    monkeypatch.setattr(
        recharge_node.ardusub,
        'check_setpoint_reached_xy',
        lambda candidate, threshold: True)
    monkeypatch.setattr(
        recharge_battery_lc,
        'call_service_with_timeout',
        lambda node, client, request: response)

    assert recharge_node._try_recharge_once() is False


def test_run_recharge_retries_until_attempt_completes(
        monkeypatch, recharge_node):
    """Verify the shared runner retries an in-progress recharge."""
    outcomes = iter([None, True])
    sleeps = []
    frequencies = []

    class Rate:
        """Record sleeps performed by the shared runner."""

        def sleep(self):
            """Record one retry delay."""
            sleeps.append(True)

    recharge_node.set_parameters([
        rclpy.parameter.Parameter('recharge_retry_rate', value=4.0)])
    monkeypatch.setattr(
        recharge_node,
        'create_rate',
        lambda frequency: frequencies.append(frequency) or Rate())
    monkeypatch.setattr(
        recharge_node, '_try_recharge_once', lambda: next(outcomes))

    result = recharge_node._run_recharge(lambda: False)

    assert result is True
    assert sleeps == [True]
    assert frequencies == [4.0]


@pytest.mark.parametrize('outcome', [True, False])
def test_run_recharge_returns_terminal_attempt_result(
        monkeypatch, recharge_node, outcome):
    """Verify the shared runner returns success and failure unchanged."""
    monkeypatch.setattr(
        recharge_node, '_try_recharge_once', lambda: outcome)

    assert recharge_node._run_recharge(lambda: False) is outcome


def test_run_recharge_stops_before_another_attempt(
        monkeypatch, recharge_node):
    """Verify an external stop request ends the shared runner."""
    attempts = []
    monkeypatch.setattr(
        recharge_node, '_try_recharge_once', lambda: attempts.append(True))

    result = recharge_node._run_recharge(lambda: True)

    assert result is None
    assert attempts == []


@pytest.mark.parametrize(
    ('outcome', 'terminal_state'),
    [(True, 'succeeded'), (False, 'aborted'), (None, 'canceled')],
)
def test_execute_recharge_maps_shared_runner_outcome(
        monkeypatch, recharge_node, outcome, terminal_state):
    """Verify action execution maps shared outcomes to terminal states."""
    class GoalHandle:
        """Capture action terminal state calls."""

        def __init__(self):
            """Create an unset goal state recorder."""
            self.is_cancel_requested = False
            self.succeeded = False
            self.aborted = False
            self.canceled_called = False

        def succeed(self):
            """Record successful completion."""
            self.succeeded = True

        def abort(self):
            """Record failed completion."""
            self.aborted = True

        def canceled(self):
            """Record canceled completion."""
            self.canceled_called = True

    goal_handle = GoalHandle()
    monkeypatch.setattr(
        recharge_node, '_run_recharge', lambda stop_requested: outcome)

    result = recharge_node._execute_recharge(goal_handle)

    assert result.success is (outcome is True)
    assert goal_handle.succeeded is (terminal_state == 'succeeded')
    assert goal_handle.aborted is (terminal_state == 'aborted')
    assert goal_handle.canceled_called is (terminal_state == 'canceled')


def test_legacy_activation_starts_shared_runner(
        executor, monkeypatch, recharge_node):
    """Verify legacy activation queues the shared recharge runner."""
    calls = []
    monkeypatch.setattr(
        recharge_node,
        '_run_recharge',
        lambda stop_requested: calls.append(stop_requested) or True)

    recharge_node.trigger_activate()

    assert wait_for_condition(lambda: bool(calls), 5.0)
    assert recharge_node._legacy_recharge_task is not None
    recharge_node.trigger_deactivate()


def test_action_server_mode_does_not_start_legacy_runner(
        monkeypatch, recharge_node):
    """Verify action mode does not queue a competing legacy task."""
    starts = []
    monkeypatch.setattr(
        recharge_node, '_start_legacy_recharge', lambda: starts.append(True))
    recharge_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server', rclpy.Parameter.Type.BOOL, True)])

    recharge_node.trigger_activate()

    assert starts == []
    assert recharge_node._legacy_recharge_task is None
    recharge_node.trigger_deactivate()


def test_deactivation_stops_legacy_runner(monkeypatch, recharge_node):
    """Verify deactivation requests that the legacy runner stop."""
    stops = []
    monkeypatch.setattr(recharge_node, '_start_legacy_recharge', lambda: None)
    monkeypatch.setattr(
        recharge_node, '_stop_legacy_recharge', lambda: stops.append(True))

    recharge_node.trigger_activate()
    recharge_node.trigger_deactivate()

    assert stops == [True]


@pytest.mark.parametrize('callback_name', ['on_cleanup', 'on_shutdown'])
def test_lifecycle_exit_stops_legacy_runner(
        monkeypatch, recharge_node, callback_name):
    """Verify cleanup and shutdown request legacy runner termination."""
    stops = []
    monkeypatch.setattr(
        recharge_node, '_stop_legacy_recharge', lambda: stops.append(True))
    monkeypatch.setattr(recharge_node.thread, 'join', lambda: None)
    monkeypatch.setattr(recharge_node.ardusub, 'destroy_node', lambda: None)

    getattr(recharge_node, callback_name)(None)

    assert stops == [True]


def test_recharge_node_has_no_legacy_timer(recharge_node):
    """Verify recharge scheduling is owned by the shared runner."""
    assert not hasattr(recharge_node, 'recharge_cb_timer')


def test_recharge_battery_lifecycle_state_set_on_activate(
        executor, monkeypatch, recharge_node):
    """Verify lifecycle state changes on activation."""
    monkeypatch.setattr(recharge_node, '_start_legacy_recharge', lambda: None)
    assert lifecycle_state_is_active(recharge_node) is False
    recharge_node.trigger_activate()
    assert lifecycle_state_is_active(recharge_node) is True
    recharge_node.trigger_deactivate()
    assert lifecycle_state_is_active(recharge_node) is False
