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

import threading
from unittest.mock import Mock

from action_test_utils import send_goal_and_wait
from action_test_utils import spin_nodes_in_executors

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import pytest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Task

from suave import follow_pipeline_lc
from suave.follow_pipeline_lc import _FollowStopReason
from suave.follow_pipeline_lc import _FollowTraversalResult
from suave.follow_pipeline_lc import PipelineFollowerLC

from suave_msgs.action import FollowPipeline
from suave_msgs.srv import GetPath


def _pose(x=0.0, y=0.0):
    """Create a Gazebo-frame pipeline pose."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    return pose


def _setpoint(x=0.0, y=0.0):
    """Create a local-frame setpoint."""
    setpoint = PoseStamped()
    setpoint.pose.position.x = x
    setpoint.pose.position.y = y
    return setpoint


class _Rate:
    """No-op rate used by deterministic traversal tests."""

    def __init__(self):
        """Create a rate with no recorded sleeps."""
        self.sleep_count = 0

    def sleep(self):
        """Record one sleep without delaying the test."""
        self.sleep_count += 1


class _Publisher:
    """Publisher test double that records messages."""

    def __init__(self):
        """Create an empty message list."""
        self.messages = []

    def publish(self, message):
        """Record a published message."""
        self.messages.append(message)


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
    """Spin helper and follow nodes in separate executors."""
    with spin_nodes_in_executors(test_node, follow_node) as executors:
        yield executors[-1]


def test_follow_pipeline_action_rejected_when_inactive(
        executor, test_node, follow_node):
    """Verify action goals are rejected when the node is not active."""
    client = ActionClient(test_node, FollowPipeline, 'follow_pipeline')
    assert client.wait_for_server(timeout_sec=5.0)

    goal = FollowPipeline.Goal()
    goal.timeout = 10.0
    goal_handle = send_goal_and_wait(client, goal)

    assert goal_handle is not None
    assert not goal_handle.accepted
    client.destroy()


def test_follow_pipeline_action_rejected_when_use_action_server_false(
        executor, test_node, follow_node):
    """Verify active goals are rejected when use_action_server=False."""
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

    # Now flip to False so the goal callback rejects.
    follow_node.set_parameters([
        rclpy.parameter.Parameter(
            'use_action_server', rclpy.Parameter.Type.BOOL, False)])

    client = ActionClient(test_node, FollowPipeline, 'follow_pipeline')
    assert client.wait_for_server(timeout_sec=5.0)

    goal = FollowPipeline.Goal()
    goal.timeout = 10.0
    goal_handle = send_goal_and_wait(client, goal)

    assert goal_handle is not None
    assert not goal_handle.accepted
    follow_node.trigger_deactivate()
    service.destroy()
    client.destroy()


def test_follow_pipeline_use_action_server_default_is_false(follow_node):
    """Verify use_action_server defaults to False."""
    assert follow_node.get_parameter(
        'use_action_server').get_parameter_value().bool_value is False


def test_follow_pipeline_can_cleanup_and_reconfigure(follow_node):
    """Verify configured resources can be destroyed and recreated."""
    follow_node.trigger_cleanup()

    assert follow_node._controller is None
    assert follow_node.get_path_service is None
    assert follow_node.pipeline_inspected_pub is None

    follow_node.trigger_configure()

    assert follow_node._controller is not None
    assert follow_node.get_path_service is not None
    assert follow_node.pipeline_inspected_pub is not None


def test_load_pipe_path_once_stores_service_path(monkeypatch, follow_node):
    """Verify the first path load stores a mutable pose list."""
    response = GetPath.Response()
    response.path.poses = [_pose(1.0, 2.0), _pose(3.0, 4.0)]
    monkeypatch.setattr(
        follow_pipeline_lc,
        'call_service_with_timeout',
        lambda node, client, request: response)

    assert follow_node._load_pipe_path_once() is True
    assert len(follow_node.pipe_path) == 2
    assert follow_node.first_inspection is False


def test_load_pipe_path_once_reuses_cached_path(monkeypatch, follow_node):
    """Verify an existing path is not requested again."""
    cached_path = [_pose()]
    follow_node.pipe_path = cached_path
    follow_node.first_inspection = False
    follow_node.distance_inspected = 10.0
    follow_node.last_point = _setpoint(10.0)

    def fail_if_called(node, client, request):
        raise AssertionError('path service should not be called')

    monkeypatch.setattr(
        follow_pipeline_lc, 'call_service_with_timeout', fail_if_called)

    assert follow_node._load_pipe_path_once() is True
    assert follow_node.pipe_path is cached_path
    assert follow_node.distance_inspected == 10.0
    assert follow_node.last_point.pose.position.x == 10.0


def test_load_pipe_path_once_reports_service_failure(
        monkeypatch, follow_node):
    """Verify path loading fails without changing inspection state."""
    monkeypatch.setattr(
        follow_pipeline_lc,
        'call_service_with_timeout',
        lambda node, client, request: None)

    assert follow_node._load_pipe_path_once() is False
    assert follow_node.first_inspection is True


def test_wait_for_setpoint_propagates_stop_reason(
        monkeypatch, follow_node):
    """Verify setpoint acquisition checks stop policy between retries."""
    rate = _Rate()
    stop_reasons = iter([None, _FollowStopReason.TIMED_OUT])
    monkeypatch.setattr(
        follow_node, '_get_pipeline_setpoint', lambda pose: None)

    setpoint, stop_reason = follow_node._wait_for_setpoint(
        _pose(), rate, lambda: next(stop_reasons))

    assert setpoint is None
    assert stop_reason is _FollowStopReason.TIMED_OUT
    assert rate.sleep_count == 1


def test_wait_until_reached_propagates_stop_reason(
        monkeypatch, follow_node):
    """Verify the reached wait loop checks the injected stop policy."""
    setpoint = _setpoint()
    monkeypatch.setattr(
        follow_node._controller,
        'is_xy_setpoint_reached',
        lambda candidate, threshold: False)

    returned_setpoint, stop_reason = \
        follow_node._wait_until_setpoint_reached(
            _pose(), setpoint, _Rate(),
            lambda: _FollowStopReason.CANCELED)

    assert returned_setpoint is setpoint
    assert stop_reason is _FollowStopReason.CANCELED


def test_wait_until_reached_refreshes_stale_setpoint(
        monkeypatch, follow_node):
    """Verify a long wait refreshes the setpoint after ten retries."""
    initial_setpoint = _setpoint()
    refreshed_setpoint = _setpoint(1.0, 1.0)
    refreshes = []
    monkeypatch.setattr(
        follow_node._controller,
        'is_xy_setpoint_reached',
        lambda candidate, threshold: candidate is refreshed_setpoint)
    monkeypatch.setattr(
        follow_node,
        '_get_pipeline_setpoint',
        lambda pose: refreshes.append(True) or refreshed_setpoint)

    returned_setpoint, stop_reason = \
        follow_node._wait_until_setpoint_reached(
            _pose(), initial_setpoint, _Rate(), lambda: None)

    assert returned_setpoint is refreshed_setpoint
    assert stop_reason is None
    assert refreshes == [True]


def test_follow_pipeline_path_publishes_progress_and_completion(
        monkeypatch, follow_node):
    """Verify shared traversal publishes distance and final completion."""
    follow_node.pipe_path = [_pose(0.0, 0.0), _pose(3.0, 4.0)]
    distance_publisher = _Publisher()
    inspected_publisher = _Publisher()
    progress = []
    follow_node.pipeline_distance_inspected_pub = distance_publisher
    follow_node.pipeline_inspected_pub = inspected_publisher
    monkeypatch.setattr(
        follow_node,
        '_wait_for_setpoint',
        lambda pose, rate, stop: (_setpoint(
            pose.position.x, pose.position.y), None))
    monkeypatch.setattr(
        follow_node,
        '_wait_until_setpoint_reached',
        lambda pose, setpoint, rate, stop: (setpoint, None))

    traversal = follow_node._follow_pipeline_path(
        lambda: None,
        on_progress=lambda distance, point: progress.append(distance))

    assert traversal.completed is True
    assert traversal.distance_inspected == 5.0
    assert follow_node.pipe_path == []
    assert [message.data for message in distance_publisher.messages] == [5.0]
    assert progress == [5.0]
    assert [message.data for message in inspected_publisher.messages] == [True]


def test_follow_pipeline_path_stop_preserves_current_waypoint(follow_node):
    """Verify stopping traversal does not consume an unreached waypoint."""
    waypoint = _pose()
    follow_node.pipe_path = [waypoint]
    inspected_publisher = _Publisher()
    follow_node.pipeline_inspected_pub = inspected_publisher

    traversal = follow_node._follow_pipeline_path(
        lambda: _FollowStopReason.LEGACY_ABORT)

    assert traversal.completed is False
    assert traversal.stop_reason is _FollowStopReason.LEGACY_ABORT
    assert follow_node.pipe_path == [waypoint]
    assert inspected_publisher.messages == []


def test_execute_follow_pipeline_aborts_when_path_loading_fails(
        monkeypatch, follow_node):
    """Verify a path-service failure aborts the action."""
    class GoalHandle:
        """Minimal goal handle for path loading failure."""

        def __init__(self):
            """Create an action goal with no timeout."""
            self.request = FollowPipeline.Goal(timeout=0.0)
            self.aborted = False

        def abort(self):
            """Record action failure."""
            self.aborted = True

    goal_handle = GoalHandle()
    monkeypatch.setattr(
        follow_node.get_path_service,
        'wait_for_service',
        lambda timeout_sec: True)
    monkeypatch.setattr(follow_node, '_load_pipe_path_once', lambda: False)

    result = follow_node._execute_follow_pipeline(goal_handle)

    assert result.success is False
    assert result.timed_out is False
    assert goal_handle.aborted is True


def test_action_timeout_is_enforced_while_setpoint_is_unavailable(
        monkeypatch, follow_node):
    """Verify action timeout remains active inside setpoint acquisition."""
    class GoalHandle:
        """Minimal goal handle for timeout execution."""

        def __init__(self):
            """Create a goal with a positive timeout."""
            self.request = FollowPipeline.Goal(timeout=0.5)
            self.is_cancel_requested = False
            self.succeeded = False

        def succeed(self):
            """Record successful terminal state."""
            self.succeeded = True

        def publish_feedback(self, feedback):
            """Accept progress feedback."""

    goal_handle = GoalHandle()
    follow_node.pipe_path = [_pose()]
    follow_node.first_inspection = False
    times = iter([0.0, 0.0, 0.0, 1.0])
    monkeypatch.setattr(
        follow_node.get_path_service,
        'wait_for_service',
        lambda timeout_sec: True)
    monkeypatch.setattr(
        follow_pipeline_lc.time, 'time', lambda: next(times))
    monkeypatch.setattr(
        follow_node, 'create_rate', lambda frequency: _Rate())
    monkeypatch.setattr(
        follow_node, '_get_pipeline_setpoint', lambda pose: None)

    result = follow_node._execute_follow_pipeline(goal_handle)

    assert result.success is True
    assert result.timed_out is True
    assert goal_handle.succeeded is True
    assert len(follow_node.pipe_path) == 1


@pytest.mark.parametrize(
    ('traversal', 'terminal_state', 'success', 'timed_out'),
    [
        (_FollowTraversalResult(True, None, 4.0, None),
         'succeeded', True, False),
        (_FollowTraversalResult(
            False, _FollowStopReason.TIMED_OUT, 3.0, None),
         'succeeded', True, True),
        (_FollowTraversalResult(
            False, _FollowStopReason.CANCELED, 2.0, None),
         'canceled', False, False),
    ],
)
def test_execute_follow_pipeline_maps_traversal_result(
        monkeypatch, follow_node, traversal, terminal_state,
        success, timed_out):
    """Verify action execution maps shared traversal outcomes."""
    class GoalHandle:
        """Capture action terminal-state calls."""

        def __init__(self):
            """Create an unset terminal state."""
            self.request = FollowPipeline.Goal(timeout=0.0)
            self.is_cancel_requested = False
            self.terminal_state = None

        def succeed(self):
            """Record action success."""
            self.terminal_state = 'succeeded'

        def canceled(self):
            """Record action cancellation."""
            self.terminal_state = 'canceled'

    goal_handle = GoalHandle()
    monkeypatch.setattr(
        follow_node.get_path_service,
        'wait_for_service',
        lambda timeout_sec: True)
    monkeypatch.setattr(follow_node, '_load_pipe_path_once', lambda: True)
    monkeypatch.setattr(
        follow_node,
        '_follow_pipeline_path',
        lambda should_stop, on_progress: traversal)

    result = follow_node._execute_follow_pipeline(goal_handle)

    assert result.success is success
    assert result.timed_out is timed_out
    assert result.distance_inspected == traversal.distance_inspected
    assert goal_handle.terminal_state == terminal_state


def test_legacy_abort_preserves_current_waypoint(monkeypatch, follow_node):
    """Verify legacy abort does not lose the current waypoint."""
    waypoint = _pose()
    follow_node.pipe_path = [waypoint]
    follow_node.abort_follow = True
    monkeypatch.setattr(follow_node, 'create_rate', lambda frequency: _Rate())

    follow_node.follow_pipeline()

    assert follow_node.pipe_path == [waypoint]
    assert follow_node.distance_inspected == 0.0


@pytest.mark.parametrize('execution', ['legacy', 'action'])
@pytest.mark.parametrize('stop_x', [0.0, 10.0, 12.0])
def test_resume_matches_uninterrupted_inspection(
        monkeypatch, follow_node, execution, stop_x):
    """Preserve cumulative distance across repeated stops inside waypoint waits."""
    distance_publisher = _Publisher()
    inspected_publisher = _Publisher()
    monkeypatch.setattr(
        follow_node, 'pipeline_distance_inspected_pub', distance_publisher)
    monkeypatch.setattr(
        follow_node, 'pipeline_inspected_pub', inspected_publisher)
    monkeypatch.setattr(follow_node, 'create_rate', lambda frequency: _Rate())
    monkeypatch.setattr(
        follow_node.get_path_service, 'wait_for_service',
        lambda timeout_sec: True)
    monkeypatch.setattr(
        follow_node, '_get_pipeline_setpoint',
        lambda pose: _setpoint(pose.position.x, pose.position.y))
    response = GetPath.Response()
    response.path.poses = [_pose(x) for x in (0.0, 10.0, 12.0, 14.0)]
    monkeypatch.setattr(
        follow_pipeline_lc, 'call_service_with_timeout',
        lambda node, client, request: response)

    def run_attempt(interrupt):
        goal = Mock()
        goal.request = FollowPipeline.Goal(timeout=0.0)
        goal.is_cancel_requested = False
        follow_node.abort_follow = False

        def reached(setpoint, threshold):
            if interrupt and setpoint.pose.position.x == stop_x:
                follow_node.abort_follow = True
                goal.is_cancel_requested = True
                return False
            return True

        monkeypatch.setattr(
            follow_node._controller, 'is_xy_setpoint_reached', reached)
        if execution == 'legacy':
            assert follow_node._load_pipe_path_once()
            follow_node.follow_pipeline()
        else:
            result = follow_node._execute_follow_pipeline(goal)
            assert result.distance_inspected == follow_node.distance_inspected
            if interrupt:
                goal.canceled.assert_called_once()
            else:
                goal.succeed.assert_called_once()
            for call in goal.publish_feedback.call_args_list:
                assert call.args[0].distance_inspected in (10.0, 12.0, 14.0)

    # Establish the reference with no pauses.
    run_attempt(False)
    reference = follow_node.distance_inspected
    assert reference == 14.0
    reference_samples = [msg.data for msg in distance_publisher.messages]

    # A new path starts a new inspection; subsequent attempts reuse it.
    follow_node.first_inspection = True
    distance_publisher.messages.clear()
    inspected_publisher.messages.clear()
    for _ in range(2):
        run_attempt(True)
        assert follow_node.pipe_path[0].position.x == stop_x
        assert follow_node.distance_inspected == (10.0 if stop_x == 12 else 0.0)
        assert inspected_publisher.messages == []

    run_attempt(False)

    assert follow_node.distance_inspected == reference
    assert [msg.data for msg in distance_publisher.messages] == reference_samples
    assert [msg.data for msg in inspected_publisher.messages] == [True]
    assert follow_node.pipe_path == []


def test_new_configuration_resets_inspection_progress(follow_node):
    """Start a new mission after cleanup instead of retaining its old path."""
    follow_node.pipe_path = [_pose(14.0)]
    follow_node.first_inspection = False
    follow_node.distance_inspected = 10.0
    follow_node.last_point = _setpoint(10.0)

    follow_node.trigger_cleanup()
    follow_node.trigger_configure()

    assert follow_node.pipe_path == []
    assert follow_node.first_inspection is True
    assert follow_node.distance_inspected == 0.0
    assert follow_node.last_point is None


@pytest.mark.parametrize('execution', ['legacy', 'action'])
@pytest.mark.parametrize('resume', [False, True])
def test_deactivation_returns_without_waiting_for_running_traversal(
        monkeypatch, follow_node, execution, resume):
    """Let recharge start immediately but serialize inspection resumption."""
    entered = threading.Event()
    release = threading.Event()
    stopped = threading.Event()
    resumed = threading.Event()
    errors = []
    follow_node.set_parameters([
        rclpy.parameter.Parameter('use_action_server', value=True)])
    monkeypatch.setattr(
        follow_node.get_path_service, 'wait_for_service',
        lambda timeout_sec: True)
    follow_node.trigger_activate()
    follow_node.first_inspection = False

    def traverse(should_stop, on_progress=None):
        entered.set()
        if not release.wait(timeout=5.0):
            raise AssertionError('test did not release traversal')
        reason = should_stop()
        if reason is None:
            errors.append('deactivation cleared the stop request too early')
        return _FollowTraversalResult(False, reason, 0.0, None)

    monkeypatch.setattr(follow_node, '_follow_pipeline_path', traverse)
    goal = Mock()
    goal.request = FollowPipeline.Goal(timeout=0.0)
    goal.is_cancel_requested = False
    if execution == 'legacy':
        task = Task(follow_node.follow_pipeline)
        follow_node.follow_task = task
    else:
        task = Task(lambda: follow_node._execute_follow_pipeline(goal))
    worker = threading.Thread(target=task)

    def deactivate():
        follow_node.trigger_deactivate()
        stopped.set()

    def reactivate():
        follow_node.trigger_activate()
        resumed.set()

    stopper = threading.Thread(target=deactivate)
    activator = threading.Thread(target=reactivate)
    worker.start()
    try:
        assert entered.wait(timeout=2.0)
        stopper.start()
        assert follow_node._abort_event.wait(timeout=2.0)
        assert stopped.wait(timeout=0.5)
        assert worker.is_alive()
        if resume:
            activator.start()
            assert not resumed.wait(timeout=0.05)
    finally:
        release.set()
        worker.join(timeout=2.0)
        if stopper.ident is not None:
            stopper.join(timeout=2.0)
        if activator.ident is not None:
            activator.join(timeout=2.0)

    assert not worker.is_alive()
    assert not stopper.is_alive()
    assert not activator.is_alive()
    assert stopped.is_set()
    assert resumed.is_set() is resume
    assert task.exception() is None
    assert errors == []


def test_stop_after_waypoint_wait_does_not_commit_progress(
        monkeypatch, follow_node):
    """Ignore a reached waypoint if deactivation occurred during its wait."""
    waypoint = _pose(12.0)
    last_point = _setpoint(10.0)
    follow_node.pipe_path = [waypoint]
    follow_node.distance_inspected = 10.0
    follow_node.last_point = last_point
    distance_publisher = _Publisher()
    inspected_publisher = _Publisher()
    monkeypatch.setattr(
        follow_node, 'pipeline_distance_inspected_pub', distance_publisher)
    monkeypatch.setattr(
        follow_node, 'pipeline_inspected_pub', inspected_publisher)
    monkeypatch.setattr(follow_node, 'create_rate', lambda frequency: _Rate())
    monkeypatch.setattr(
        follow_node, '_get_pipeline_setpoint', lambda pose: _setpoint(12.0))

    def reached_after_stop(pose, setpoint, rate, should_stop):
        follow_node.abort_follow = True
        return setpoint, None

    monkeypatch.setattr(
        follow_node, '_wait_until_setpoint_reached', reached_after_stop)

    follow_node.follow_pipeline()

    assert follow_node.pipe_path == [waypoint]
    assert follow_node.distance_inspected == 10.0
    assert follow_node.last_point is last_point
    assert distance_publisher.messages == []
    assert inspected_publisher.messages == []
