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

import time
import threading

import pytest
import rclpy

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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
    """Spin the helper and recharge nodes in separate executors to avoid reentrancy."""
    helper_ex = MultiThreadedExecutor()
    helper_ex.add_node(test_node)
    helper_thread = threading.Thread(target=helper_ex.spin, daemon=True)
    helper_thread.start()

    node_ex = MultiThreadedExecutor()
    node_ex.add_node(recharge_node)
    node_thread = threading.Thread(target=node_ex.spin, daemon=True)
    node_thread.start()
    try:
        yield node_ex
    finally:
        helper_ex.shutdown()
        node_ex.shutdown()
        helper_thread.join(timeout=2.0)
        node_thread.join(timeout=2.0)


def test_recharge_battery_action_rejected_when_inactive(
        executor, test_node, recharge_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(
        test_node, RechargeBatteryAction, 'recharge_battery')
    assert client.wait_for_server(timeout_sec=5.0)

    future = client.send_goal_async(RechargeBatteryAction.Goal())
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    client.destroy()


def test_recharge_battery_action_rejected_when_use_action_server_false(
        executor, test_node, recharge_node):
    """Verify action goals are rejected when active but use_action_server=False."""
    recharge_node.trigger_activate()
    client = ActionClient(
        test_node, RechargeBatteryAction, 'recharge_battery')
    assert client.wait_for_server(timeout_sec=5.0)

    future = client.send_goal_async(RechargeBatteryAction.Goal())
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)

    assert future.done()
    assert not future.result().accepted
    recharge_node.trigger_deactivate()
    client.destroy()


def test_recharge_battery_use_action_server_default_is_false(recharge_node):
    """Verify use_action_server defaults to False."""
    assert recharge_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False


def test_recharge_battery_active_flag_set_on_activate(
        executor, recharge_node):
    """Verify the active flag is set on activation (backward-compat mode)."""
    assert recharge_node.active is False
    recharge_node.trigger_activate()
    assert recharge_node.active is True
    recharge_node.trigger_deactivate()
    assert recharge_node.active is False
