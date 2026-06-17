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

"""Tests for the follow pipeline lifecycle node action server."""

import time
import threading

import pytest
import rclpy

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from suave.follow_pipeline_lc import PipelineFollowerLC
from suave_msgs.action import FollowPipeline


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
    node = Node('test_follow_pipeline_helper')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def follow_node():
    """Create the pipeline follower lifecycle node under test."""
    node = PipelineFollowerLC('test_follow_pipeline_lc')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def executor(test_node, follow_node):
    """Spin the helper and follow nodes in separate executors to avoid reentrancy."""
    helper_ex = MultiThreadedExecutor()
    helper_ex.add_node(test_node)
    helper_thread = threading.Thread(target=helper_ex.spin, daemon=True)
    helper_thread.start()

    node_ex = MultiThreadedExecutor()
    node_ex.add_node(follow_node)
    node_thread = threading.Thread(target=node_ex.spin, daemon=True)
    node_thread.start()
    try:
        yield node_ex
    finally:
        helper_ex.shutdown()
        node_ex.shutdown()
        helper_thread.join(timeout=2.0)
        node_thread.join(timeout=2.0)


def test_follow_pipeline_action_rejected_when_inactive(
        executor, test_node, follow_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(test_node, FollowPipeline, 'follow_pipeline')
    assert client.wait_for_server(timeout_sec=5.0)

    goal = FollowPipeline.Goal()
    goal.timeout = 10.0
    future = client.send_goal_async(goal)
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    client.destroy()


def test_follow_pipeline_action_rejected_when_use_action_server_false(
        executor, test_node, follow_node):
    """Verify action goals are rejected when active but use_action_server=False."""
    from suave_msgs.srv import GetPath

    def _get_path_cb(request, response):
        return response

    # Provide the service so on_activate can pass its wait_for_service check.
    service = test_node.create_service(
        GetPath, 'pipeline/get_path', _get_path_cb)

    # Activate with use_action_server=True so on_activate skips call_service,
    # avoiding executor reentrancy.
    follow_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server', rclpy.Parameter.Type.BOOL, True)])
    follow_node.trigger_activate()

    # Now flip to False — _goal_callback will reject.
    follow_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server', rclpy.Parameter.Type.BOOL, False)])

    client = ActionClient(test_node, FollowPipeline, 'follow_pipeline')
    assert client.wait_for_server(timeout_sec=5.0)

    goal = FollowPipeline.Goal()
    goal.timeout = 10.0
    future = client.send_goal_async(goal)
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    follow_node.trigger_deactivate()
    service.destroy()
    client.destroy()


def test_follow_pipeline_use_action_server_default_is_false(follow_node):
    """Verify use_action_server defaults to False."""
    assert follow_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False
