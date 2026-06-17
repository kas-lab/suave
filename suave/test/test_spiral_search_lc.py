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
import threading

import pytest
import rclpy

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

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
    """Spin the helper and spiral nodes in separate executors to avoid reentrancy."""
    ex = MultiThreadedExecutor()
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    try:
        ex.add_node(spiral_node)
        ex.add_node(test_node)
        yield ex
    finally:
        ex.shutdown()
        t.join(timeout=2.0)
    # helper_ex = MultiThreadedExecutor()
    # helper_ex.add_node(test_node)
    # helper_thread = threading.Thread(target=helper_ex.spin, daemon=True)
    # helper_thread.start()

    # node_ex = MultiThreadedExecutor()
    # node_ex.add_node(spiral_node)
    # node_thread = threading.Thread(target=node_ex.spin, daemon=True)
    # node_thread.start()
    # try:
    #     yield node_ex
    # finally:
    #     helper_ex.shutdown()
    #     node_ex.shutdown()
    #     helper_thread.join(timeout=2.0)
    #     node_thread.join(timeout=2.0)


def test_spiral_search_action_rejected_when_inactive(
        executor, test_node, spiral_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    future = client.send_goal_async(SpiralSearch.Goal())
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    client.destroy()


def test_spiral_search_action_rejected_when_use_action_server_false(
        executor, test_node, spiral_node):
    """Verify action goals are rejected when active but use_action_server=False."""
    spiral_node.trigger_activate()
    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    future = client.send_goal_async(SpiralSearch.Goal())
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    spiral_node.trigger_deactivate()
    client.destroy()


def test_spiral_search_use_action_server_false_enables_spiral_on_activate(
        executor, spiral_node):
    """Verify use_action_server=False starts _enabled=True on activation."""
    assert spiral_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False

    spiral_node.trigger_activate()
    assert spiral_node._enabled is True
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
    """Verify action succeeds with pipeline_found=True when pipeline/detected fires."""
    spiral_node.trigger_activate()

    client = ActionClient(test_node, SpiralSearch, 'spiral_search')
    assert client.wait_for_server(timeout_sec=5.0)

    spiral_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server',
            rclpy.Parameter.Type.BOOL,
            True)])

    def _feedback_cb(msg):
        pass

    goal_future = client.send_goal_async(
        SpiralSearch.Goal(), feedback_callback=_feedback_cb)
    deadline = time.time() + 5.0
    while not goal_future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert goal_future.result().accepted

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

    result_future = goal_future.result().get_result_async()
    deadline = time.time() + 10.0
    while not result_future.done() and time.time() < deadline:
        time.sleep(0.1)

    assert result_future.done()
    assert result_future.result().result.pipeline_found is True
    assert result_future.result().result.time_spent >= 0.0

    spiral_node.trigger_deactivate()
    pub.destroy()
    client.destroy()
