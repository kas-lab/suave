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

"""Tests for the recover thrusters lifecycle node."""
import threading
import time

from action_test_utils import send_goal_and_wait
from action_test_utils import spin_nodes_in_executors
from action_test_utils import wait_for_action_result
from action_test_utils import wait_for_condition

import pytest

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from suave import recover_thrusters_lc
from suave.recover_thrusters_lc import RecoverThrustersLC
from suave.ros_service_utils import call_service_with_timeout

from suave_msgs.action import RecoverThrusters


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
    node = Node('test_recover_thrusters_helper')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def recover_node():
    """Create the recover thrusters lifecycle node under test."""
    node = RecoverThrustersLC('test_recover_thrusters_lc')
    try:
        yield node
    finally:
        node.destroy_node()


@pytest.fixture
def executor(test_node, recover_node):
    """Spin helper and recover nodes in separate executors."""
    with spin_nodes_in_executors(test_node, recover_node) as executors:
        yield executors[-1]


def test_recover_thrusters_uses_configured_client_and_waits_for_response(
        executor, test_node, recover_node):
    """Verify the stored client completes a real ROS service call."""
    requests = []

    def _set_parameters_cb(request, response):
        requests.append(request)
        response.results.append(SetParametersResult(successful=True))
        return response

    service = test_node.create_service(
        SetParameters,
        '/mavros/param/set_parameters',
        _set_parameters_cb
    )

    parameter = Parameter()
    parameter.name = 'SERVO1_FUNCTION'
    parameter.value = ParameterValue(
        type=ParameterType.PARAMETER_INTEGER,
        integer_value=33
    )
    request = SetParameters.Request(parameters=[parameter])

    response = call_service_with_timeout(
        recover_node, recover_node.set_parameters_service, request)

    assert response is not None
    assert len(response.results) == 1
    assert response.results[0].successful is True
    assert len(requests) == 1
    assert requests[0].parameters[0].name == 'SERVO1_FUNCTION'
    assert requests[0].parameters[0].value.integer_value == 33
    assert recover_node.set_parameters_service is not None

    test_node.destroy_service(service)


def test_destroy_set_parameters_service_releases_configured_client(
        recover_node):
    """Verify cleanup destroys the configured parameter client."""
    client = recover_node.set_parameters_service

    recover_node._destroy_configured_entities()

    assert client is not None
    assert recover_node.set_parameters_service is None


def test_recover_thrusters_returns_false_when_parameter_update_rejected(
        monkeypatch, recover_node):
    """Verify recovery fails when MAVROS rejects parameter writes."""
    requests = []

    def _call_service(
            node, client, request, stop_requested=None):
        requests.append(request)
        response = SetParameters.Response()
        response.results.append(SetParametersResult(successful=False))
        return response

    monkeypatch.setattr(
        recover_node,
        '_wait_interruptibly',
        lambda duration, stop_requested: True,
    )
    monkeypatch.setattr(
        recover_thrusters_lc, 'call_service_with_timeout', _call_service)

    assert recover_node._recover_thrusters() is False
    assert len(requests) == 6


def test_execute_recover_aborts_when_recovery_fails(
        monkeypatch, recover_node):
    """Verify action execution reports failed recovery as failure."""
    class GoalHandle:
        """Capture action terminal state calls."""

        def __init__(self):
            """Create an unset goal state recorder."""
            self.succeeded = False
            self.aborted = False

        def succeed(self):
            """Record successful completion."""
            self.succeeded = True

        def abort(self):
            """Record failed completion."""
            self.aborted = True

    goal_handle = GoalHandle()
    monkeypatch.setattr(
        recover_node, '_recover_thrusters', lambda stop_requested=None: False)

    result = recover_node._execute_recover(goal_handle)

    assert result.success is False
    assert goal_handle.aborted is True
    assert goal_handle.succeeded is False


def test_recover_thrusters_action_rejected_when_inactive(
        executor, test_node, recover_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(test_node, RecoverThrusters, 'recover_thrusters')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, RecoverThrusters.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    client.destroy()


def test_recover_thrusters_action_rejected_when_use_action_server_false(
        executor, test_node, recover_node):
    """Verify active goals are rejected when use_action_server=False."""
    recover_node.trigger_activate()
    client = ActionClient(test_node, RecoverThrusters, 'recover_thrusters')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, RecoverThrusters.Goal())

    assert goal_handle is not None
    assert not goal_handle.accepted
    recover_node.trigger_deactivate()
    assert recover_node.recover_task.done()
    assert not recover_node.recover_task.cancelled()
    client.destroy()


def test_recover_thrusters_action_accepted_when_active(
        executor, test_node, recover_node):
    """Verify active goals are accepted when use_action_server=True."""
    def _set_parameters_cb(request, response):
        response.results.append(SetParametersResult(successful=True))
        return response

    service = test_node.create_service(
        SetParameters, '/mavros/param/set_parameters', _set_parameters_cb)

    recover_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server', rclpy.Parameter.Type.BOOL, True)])
    recover_node.trigger_activate()
    client = ActionClient(test_node, RecoverThrusters, 'recover_thrusters')
    assert client.wait_for_server(timeout_sec=5.0)

    goal_handle = send_goal_and_wait(client, RecoverThrusters.Goal())

    assert goal_handle is not None
    assert goal_handle.accepted

    result = wait_for_action_result(goal_handle, timeout_sec=30.0)

    assert result is not None
    assert result.result.success is True

    recover_node.trigger_deactivate()
    test_node.destroy_service(service)
    client.destroy()


def test_recover_thrusters_no_action_server_mode_starts_on_activate(
        executor, test_node, recover_node):
    """Verify use_action_server=False starts recovery on activation."""
    started = []
    original = recover_node._recover_thrusters

    def _mock_recover(stop_requested=None):
        started.append(True)

    recover_node._recover_thrusters = _mock_recover
    recover_node.trigger_activate()

    assert wait_for_condition(lambda: bool(started), 5.0), (
        '_recover_thrusters() was not called on activation')
    recover_node._recover_thrusters = original
    recover_node.trigger_deactivate()


def test_interruptible_wait_stops_when_abort_is_requested(recover_node):
    """Verify lifecycle deactivation interrupts recovery delays."""
    recover_node._abort_event.set()

    started = time.monotonic()
    completed = recover_node._wait_interruptibly(
        10.0,
        recover_node._abort_event.is_set,
    )

    assert completed is False
    assert time.monotonic() - started < 0.5


def test_service_wait_cancels_future_when_stop_is_requested():
    """Verify cooperative stopping cancels a pending service future."""
    stop_event = threading.Event()

    class Future:
        """Pending service future test double."""

        def __init__(self):
            """Create an uncanceled pending future."""
            self.canceled = False

        def done(self):
            """Keep the response pending."""
            return False

        def cancel(self):
            """Record cancellation."""
            self.canceled = True

    class Client:
        """Available service client test double."""

        srv_name = 'test/service'

        def __init__(self, future):
            """Store the response future."""
            self.future = future

        def wait_for_service(self, timeout_sec):
            """Report the service as available."""
            return True

        def call_async(self, request):
            """Return the pending response future."""
            return self.future

    class Executor:
        """Executor test double that requests a stop after one poll."""

        def spin_until_future_complete(self, future, timeout_sec):
            """Request cooperative stopping."""
            stop_event.set()

    class Node:
        """Service-helper node test double."""

        executor = Executor()

        class Logger:
            """Logger test double."""

            def error(self, message):
                """Accept an error message."""

        def get_logger(self):
            """Return a logger test double."""
            return self.Logger()

    future = Future()
    response = call_service_with_timeout(
        Node(),
        Client(future),
        object(),
        stop_requested=stop_event.is_set,
    )

    assert response is None
    assert future.canceled is True
