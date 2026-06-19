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

"""Tests for the spiral search lifecycle node action server."""

import time

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from action_test_utils import send_goal_and_wait
from action_test_utils import spin_nodes_in_executors
from action_test_utils import wait_for_action_result
from suave.spiral_search_lc import SpiralSearcherLC
from suave_msgs.action import SpiralSearch


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
    node = Node('test_spiral_search_helper')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def spiral_node():
    """Create the spiral searcher lifecycle node under test."""
    node = SpiralSearcherLC('test_spiral_search_lc')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def executor(test_node, spiral_node):
    """Spin helper and spiral nodes in separate executors."""
    with spin_nodes_in_executors(test_node, spiral_node) as executors:
        yield executors[-1]


def test_spiral_search_action_rejected_when_inactive(
        executor, test_node, spiral_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, SpiralSearch.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    client.destroy()


def test_spiral_search_action_rejected_when_use_action_server_false(
        executor, test_node, spiral_node):
    """Verify active goals are rejected when use_action_server=False."""
    spiral_node.trigger_activate()
    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, SpiralSearch.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    spiral_node.trigger_deactivate()
    client.destroy()


def test_start_spiral_does_not_reset_pipeline_detection(spiral_node):
    """Verify _start_spiral does not clear pipeline detection."""
    spiral_node._pipeline_detected = True

    spiral_node._start_spiral()

    assert spiral_node._enabled is True
    assert spiral_node._pipeline_detected is True


def test_reset_spiral_state_clears_pipeline_detection(spiral_node):
    """Verify _reset_spiral_state clears pipeline detection."""
    spiral_node._pipeline_detected = True

    spiral_node._reset_spiral_state()
    spiral_node._start_spiral()

    assert spiral_node._enabled is True
    assert spiral_node._pipeline_detected is False


def test_stop_spiral_disables_movement(spiral_node):
    """Verify spiral stop disables timer-driven movement."""
    spiral_node._start_spiral()

    spiral_node._stop_spiral()

    assert spiral_node._enabled is False


def test_action_cancellation_stops_spiral(spiral_node):
    """Verify action cancellation always disables spiral movement."""
    class GoalHandle:
        """Minimal canceled goal handle for action execution."""

        is_cancel_requested = True

        def __init__(self):
            """Create a goal handle that has not been canceled."""
            self.canceled_called = False

        def canceled(self):
            """Record action cancellation."""
            self.canceled_called = True

    goal_handle = GoalHandle()

    result = spiral_node._execute_spiral_search(goal_handle)

    assert result.pipeline_found is False
    assert goal_handle.canceled_called is True
    assert spiral_node._enabled is False


def test_spiral_search_use_action_server_false_enables_spiral_on_activate(
        executor, spiral_node):
    """Verify use_action_server=False starts _enabled=True on activation."""
    assert spiral_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False

    spiral_node.spiral_center_x = 10.0
    spiral_node.spiral_center_y = 20.0
    spiral_node.trigger_activate()
    assert spiral_node._enabled is True
    assert spiral_node.spiral_center_x is None
    assert spiral_node.spiral_center_y is None
    spiral_node.trigger_deactivate()
    assert spiral_node._enabled is False


def test_spiral_search_use_action_server_true_does_not_enable_on_activate(
        executor, spiral_node):
    """Verify use_action_server=True does not start behavior on activation."""
    spiral_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server',
            rclpy.Parameter.Type.BOOL,
            True)])

    spiral_node.trigger_activate()
    assert spiral_node._enabled is False
    spiral_node.trigger_deactivate()

    spiral_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server',
            rclpy.Parameter.Type.BOOL,
            False)])


def test_spiral_search_action_completes_when_pipeline_detected(
        executor, test_node, spiral_node):
    """Verify the action succeeds when pipeline/detected fires."""
    spiral_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server',
            rclpy.Parameter.Type.BOOL,
            True)])
    spiral_node.trigger_activate()

    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    def _feedback_cb(msg):
        pass

    goal_handle = send_goal_and_wait(
        client, SpiralSearch.Goal(), feedback_callback=_feedback_cb)

    assert goal_handle is not None
    assert goal_handle.accepted

    # Give the execute callback time to start
    time.sleep(0.2)

    # Simulate pipeline detection
    pub = test_node.create_publisher(Bool, 'pipeline/detected', 10)
    time.sleep(0.1)
    msg = Bool()
    msg.data = True
    for _ in range(5):
        pub.publish(msg)
        time.sleep(0.1)

    result = wait_for_action_result(goal_handle, timeout_sec=10.0)

    assert result is not None
    assert result.result.pipeline_found is True
    assert result.result.time_spent >= 0.0
    assert spiral_node._enabled is False

    spiral_node.trigger_deactivate()
    pub.destroy()
    client.destroy()


def test_spiral_timer_is_stored_for_lifecycle_cleanup(spiral_node):
    """Verify configuration stores the timer under the managed attribute."""
    assert spiral_node._timer is not None
    assert not hasattr(spiral_node, '_timer_')


def test_spiral_publish_passes_explicit_altitude(
        monkeypatch, spiral_node):
    """Verify spiral search passes its corrected altitude explicitly."""
    local_pose = PoseStamped()
    local_pose.pose.position.x = 10.0
    local_pose.pose.position.y = 20.0
    spiral_node._controller._local_pose = local_pose
    spiral_node._enabled = True
    spiral_node.z_delta = -0.25
    calls = []

    def publish_xy_setpoint(x, y, altitude):
        calls.append((x, y, altitude))
        return PoseStamped()

    monkeypatch.setattr(
        spiral_node._controller,
        'publish_xy_setpoint',
        publish_xy_setpoint,
    )

    spiral_node.publish()

    assert calls
    assert calls[-1][2] == pytest.approx(1.75)


def test_spiral_node_can_cleanup_and_reconfigure(spiral_node):
    """Verify spiral resources can be destroyed and recreated."""
    spiral_node.trigger_cleanup()

    assert spiral_node._controller is None
    assert spiral_node._pipeline_detected_sub is None
    assert spiral_node._timer is None

    spiral_node.trigger_configure()

    assert spiral_node._controller is not None
    assert spiral_node._pipeline_detected_sub is not None
    assert spiral_node._timer is not None
