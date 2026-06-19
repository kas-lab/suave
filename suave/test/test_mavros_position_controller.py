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

"""Tests for the MAVROS position controller."""

import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from suave.mavros_position_controller import MavrosPositionController


@pytest.fixture
def rclpy_context():
    """Create an isolated rclpy context."""
    context = rclpy.context.Context()
    rclpy.init(context=context)
    try:
        yield context
    finally:
        rclpy.shutdown(context=context)


@pytest.fixture
def controller(rclpy_context):
    """Create a node and position controller."""
    node = Node('test_mavros_position_controller', context=rclpy_context)
    position_controller = MavrosPositionController(
        node, -20.0, MutuallyExclusiveCallbackGroup())
    try:
        yield node, position_controller
    finally:
        position_controller.destroy()
        node.destroy_node()


def _pose_stamped(x: float, y: float, z: float) -> PoseStamped:
    """Create a stamped pose with the requested position."""
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose


def test_controller_starts_without_local_pose(controller):
    """Verify local position is unavailable until MAVROS publishes it."""
    _, position_controller = controller

    assert position_controller.local_pose is None
    assert position_controller.has_local_pose is False
    assert position_controller.publish_gazebo_setpoint(Pose(), 1.25) is None
    assert position_controller.is_setpoint_reached(PoseStamped(), 0.1) is False
    assert position_controller.is_xy_setpoint_reached(
        PoseStamped(), 0.1) is False


def test_gazebo_setpoint_uses_explicit_altitude_without_mutating_input(
        controller, monkeypatch):
    """Verify Gazebo setpoint conversion and input ownership."""
    _, position_controller = controller
    published = []
    monkeypatch.setattr(
        position_controller._setpoint_publisher,
        'publish',
        published.append,
    )
    position_controller._local_pose_callback(_pose_stamped(0.0, 0.0, -20.0))
    gazebo_pose = Pose(position=Point(x=3.0, y=4.0, z=-7.0))

    setpoint = position_controller.publish_gazebo_setpoint(
        gazebo_pose, 2.5)

    assert setpoint is not None
    assert setpoint.pose.position.x == pytest.approx(3.0)
    assert setpoint.pose.position.y == pytest.approx(4.0)
    assert setpoint.pose.position.z == pytest.approx(-17.5)
    assert setpoint.pose.orientation.w == pytest.approx(1.0)
    assert setpoint.header.stamp.sec > 0
    assert gazebo_pose.position.z == pytest.approx(-7.0)
    assert published == [setpoint]


def test_reachability_checks_xyz_and_xy_separately(controller):
    """Verify XYZ checks include Z while XY checks ignore it."""
    _, position_controller = controller
    position_controller._local_pose_callback(_pose_stamped(1.0, 2.0, 3.0))
    setpoint = _pose_stamped(1.2, 2.2, 10.0)

    assert position_controller.is_setpoint_reached(setpoint, 0.3) is False
    assert position_controller.is_xy_setpoint_reached(setpoint, 0.3) is True


def test_destroy_is_idempotent(controller):
    """Verify controller resources can be destroyed repeatedly."""
    _, position_controller = controller

    position_controller.destroy()
    position_controller.destroy()

    assert position_controller._local_position_subscription is None
    assert position_controller._setpoint_publisher is None


def test_subscription_and_publication_use_owner_executor(rclpy_context):
    """Verify position traffic flows through the owner's executor."""
    owner = Node('position_owner', context=rclpy_context)
    peer = Node('position_peer', context=rclpy_context)
    position_controller = MavrosPositionController(
        owner, -20.0, MutuallyExclusiveCallbackGroup())
    pose_publisher = peer.create_publisher(
        PoseStamped, '/mavros/local_position/pose', 10)
    received_setpoints = []
    setpoint_subscription = peer.create_subscription(
        PoseStamped,
        '/mavros/setpoint_position/local',
        received_setpoints.append,
        10,
    )
    executor = MultiThreadedExecutor(context=rclpy_context)
    executor.add_node(owner)
    executor.add_node(peer)

    try:
        deadline = time.monotonic() + 3.0
        while (not position_controller.has_local_pose and
               time.monotonic() < deadline):
            pose_publisher.publish(_pose_stamped(1.0, 2.0, -18.0))
            executor.spin_once(timeout_sec=0.05)

        assert position_controller.has_local_pose
        position_controller.publish_xy_setpoint(4.0, 5.0, 1.25)

        deadline = time.monotonic() + 3.0
        while not received_setpoints and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        assert len(received_setpoints) == 1
        assert received_setpoints[0].pose.position.z == pytest.approx(-18.75)
    finally:
        position_controller.destroy()
        peer.destroy_subscription(setpoint_subscription)
        peer.destroy_publisher(pose_publisher)
        executor.remove_node(owner)
        executor.remove_node(peer)
        executor.shutdown()
        owner.destroy_node()
        peer.destroy_node()


def test_position_callback_runs_while_default_group_is_occupied(
        rclpy_context):
    """Verify the dedicated group prevents position callback starvation."""
    owner = Node('occupied_position_owner', context=rclpy_context)
    peer = Node('occupied_position_peer', context=rclpy_context)
    position_controller = MavrosPositionController(
        owner, -20.0, MutuallyExclusiveCallbackGroup())
    pose_publisher = peer.create_publisher(
        PoseStamped, '/mavros/local_position/pose', 10)
    callback_started = threading.Event()
    release_callback = threading.Event()

    def occupy_default_group():
        callback_started.set()
        release_callback.wait(timeout=3.0)

    timer = owner.create_timer(0.01, occupy_default_group)
    executor = MultiThreadedExecutor(
        num_threads=2,
        context=rclpy_context,
    )
    executor.add_node(owner)
    executor.add_node(peer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        assert callback_started.wait(timeout=3.0)
        deadline = time.monotonic() + 3.0
        while (not position_controller.has_local_pose and
               time.monotonic() < deadline):
            pose_publisher.publish(_pose_stamped(1.0, 2.0, -18.0))
            time.sleep(0.05)

        assert position_controller.has_local_pose
    finally:
        release_callback.set()
        executor.shutdown()
        spin_thread.join(timeout=3.0)
        position_controller.destroy()
        owner.destroy_timer(timer)
        peer.destroy_publisher(pose_publisher)
        owner.destroy_node()
        peer.destroy_node()
