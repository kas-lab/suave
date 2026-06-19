# Copyright 2023 Gustavo Rezende Silva
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

"""Lifecycle node that follows a detected pipeline."""

from dataclasses import dataclass
from enum import Enum
import math
import time
import rclpy
import threading
from typing import Callable
from typing import Optional

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Rate

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from suave_msgs.action import FollowPipeline
from suave_msgs.srv import GetPath
from suave.action_server_utils import accept_cancel
from suave.action_server_utils import make_goal_callback
from suave.action_server_utils import use_action_server
from suave.action_server_utils import wait_for_action_completion
from suave.mavros_position_controller import MavrosPositionController
from suave.ros_service_utils import call_service_with_timeout

from std_msgs.msg import Bool
from std_msgs.msg import Float32


class _FollowStopReason(Enum):
    """Reasons why pipeline traversal can stop before completion."""

    CANCELED = 'canceled'
    TIMED_OUT = 'timed_out'
    LEGACY_ABORT = 'legacy_abort'
    DEACTIVATED = 'deactivated'


@dataclass
class _FollowTraversalResult:
    """Result returned by the shared pipeline traversal."""

    completed: bool
    stop_reason: Optional[_FollowStopReason]
    distance_inspected: float
    last_point: Optional[PoseStamped]


class PipelineFollowerLC(Node):
    """Lifecycle node that follows a detected pipeline path."""

    def __init__(self, node_name, **kwargs):
        """Create the pipeline follower node."""
        super().__init__(node_name, **kwargs)
        self.abort_follow = False
        self.distance_inspected = 0
        self.first_inspection = True
        self._action_server = None
        self._abort_event = threading.Event()
        self._goal_executing = threading.Event()
        self._position_callback_group = MutuallyExclusiveCallbackGroup()
        self._path_callback_group = MutuallyExclusiveCallbackGroup()
        self._controller = None
        self.get_path_timer = None
        self.get_path_service = None
        self.pipeline_inspected_pub = None
        self.pipeline_distance_inspected_pub = None
        self.declare_parameter('ground_depth_gz', -20.0)
        self.declare_parameter('altitude', 1.25)
        self.declare_parameter('system_mode_marker', '__DEFAULT__')
        self.declare_parameter('use_action_server', False)
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Create configured publishers, clients, and the action server."""
        self.get_logger().info("on_configure() is called.")

        self.get_path_timer = self.create_rate(5)
        self.get_path_service = self.create_client(
            GetPath,
            'pipeline/get_path',
            callback_group=self._path_callback_group)

        self.pipeline_inspected_pub = self.create_lifecycle_publisher(
            Bool, 'pipeline/inspected', 10)

        self.pipeline_distance_inspected_pub = self.create_publisher(
            Float32, 'pipeline/distance_inspected', 10)

        ground_depth = self.get_parameter(
            'ground_depth_gz').get_parameter_value().double_value
        self._controller = MavrosPositionController(
            self,
            ground_depth,
            self._position_callback_group,
        )

        self._action_server = ActionServer(
            self,
            FollowPipeline,
            'follow_pipeline',
            execute_callback=self._execute_follow_pipeline,
            goal_callback=make_goal_callback(self),
            cancel_callback=accept_cancel,
        )

        self.get_logger().info("on_configure() completed")
        return TransitionCallbackReturn.SUCCESS

    def _make_result(
            self, success: bool, distance_inspected: float = 0.0,
            timed_out: bool = False) -> FollowPipeline.Result:
        """Create a FollowPipeline action result."""
        result = FollowPipeline.Result()
        result.distance_inspected = distance_inspected
        result.success = success
        result.timed_out = timed_out
        return result

    def _load_pipe_path_once(self) -> bool:
        """Load and store the pipeline path when it has not been loaded yet."""
        if not self.first_inspection:
            return True
        response = call_service_with_timeout(
            self, self.get_path_service, GetPath.Request())
        if response is None:
            return False
        self.pipe_path = list(response.path.poses)
        self.first_inspection = False
        return True

    def _get_pipeline_setpoint(
            self, gz_pose: Pose) -> Optional[PoseStamped]:
        """Request a fixed-altitude setpoint for a pipeline pose."""
        if self._controller is None:
            return None
        altitude = self.get_parameter(
            'altitude').get_parameter_value().double_value
        return self._controller.publish_gazebo_setpoint(
            gz_pose, altitude)

    def _wait_for_setpoint(
            self, gz_pose: Pose, rate: Rate,
            should_stop: Callable[[], Optional[_FollowStopReason]]
            ) -> tuple[Optional[PoseStamped], Optional[_FollowStopReason]]:
        """Wait for a setpoint or return the requested stop reason."""
        while True:
            stop_reason = should_stop()
            if stop_reason is not None:
                return None, stop_reason
            setpoint = self._get_pipeline_setpoint(gz_pose)
            if setpoint is not None:
                return setpoint, None
            rate.sleep()

    def _wait_until_setpoint_reached(
            self, gz_pose: Pose, setpoint: PoseStamped, rate: Rate,
            should_stop: Callable[[], Optional[_FollowStopReason]]
            ) -> tuple[Optional[PoseStamped], Optional[_FollowStopReason]]:
        """Wait until a setpoint is reached or a stop is requested."""
        if self._controller is None:
            return None, None
        count = 0
        while not self._controller.is_xy_setpoint_reached(setpoint, 0.5):
            stop_reason = should_stop()
            if stop_reason is not None:
                return setpoint, stop_reason
            if count > 10:
                setpoint_returned, stop_reason = self._wait_for_setpoint(
                    gz_pose, rate, should_stop)
                if setpoint_returned is None:
                    return None, stop_reason
                setpoint = setpoint_returned
                if stop_reason is not None:
                    return setpoint, stop_reason
            count += 1
            rate.sleep()
        return setpoint, None

    def _publish_distance_progress(self, distance_inspected: float) -> None:
        """Publish the current inspected pipeline distance."""
        distance_message = Float32()
        distance_message.data = distance_inspected
        self.pipeline_distance_inspected_pub.publish(distance_message)

    def _publish_pipeline_inspected(self) -> None:
        """Publish that the complete pipeline path was inspected."""
        inspected_message = Bool()
        inspected_message.data = True
        self.pipeline_inspected_pub.publish(inspected_message)

    def _follow_pipeline_path(
            self,
            should_stop: Callable[[], Optional[_FollowStopReason]],
            on_progress: Optional[
                Callable[[float, PoseStamped], None]] = None,
            ) -> _FollowTraversalResult:
        """Follow the path using caller-provided stop and progress policy."""
        rate = self.create_rate(0.5)
        last_point = None
        distance_inspected = 0.0

        while self.pipe_path:
            stop_reason = should_stop()
            if stop_reason is not None:
                return _FollowTraversalResult(
                    False, stop_reason, distance_inspected, last_point)

            gz_pose = self.pipe_path[0]
            setpoint, stop_reason = self._wait_for_setpoint(
                gz_pose, rate, should_stop)
            if stop_reason is not None:
                return _FollowTraversalResult(
                    False, stop_reason, distance_inspected, last_point)

            setpoint, stop_reason = self._wait_until_setpoint_reached(
                gz_pose, setpoint, rate, should_stop)
            if stop_reason is not None:
                return _FollowTraversalResult(
                    False, stop_reason, distance_inspected, last_point)

            self.pipe_path.pop(0)
            if last_point is not None and setpoint is not None:
                distance_inspected += self.calc_distance(
                    last_point, setpoint)
                self._publish_distance_progress(distance_inspected)
                if on_progress is not None:
                    on_progress(distance_inspected, setpoint)
            last_point = setpoint

        self._publish_pipeline_inspected()
        self.get_logger().info('Follow pipeline completed')
        return _FollowTraversalResult(
            True, None, distance_inspected, last_point)

    def _execute_follow_pipeline(
            self, goal_handle: ServerGoalHandle) -> FollowPipeline.Result:
        """Follow the path with timeout, feedback, and cancellation support."""
        self._goal_executing.set()
        try:
            timeout = goal_handle.request.timeout
            start_time = time.time()

            if not self.get_path_service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    'pipeline/get_path service not available')
                result = self._make_result(False)
                goal_handle.abort()
                return result

            if not self._load_pipe_path_once():
                result = self._make_result(False)
                goal_handle.abort()
                return result

            def should_stop() -> Optional[_FollowStopReason]:
                """Return the current action stop reason, if any."""
                if self._abort_event.is_set():
                    return _FollowStopReason.DEACTIVATED
                elapsed = time.time() - start_time
                if timeout > 0.0 and elapsed >= timeout:
                    return _FollowStopReason.TIMED_OUT
                if goal_handle.is_cancel_requested:
                    return _FollowStopReason.CANCELED
                return None

            def publish_feedback(
                    distance_inspected: float, setpoint: PoseStamped) -> None:
                """Publish action feedback for traversal progress."""
                feedback = FollowPipeline.Feedback()
                feedback.distance_inspected = distance_inspected
                goal_handle.publish_feedback(feedback)

            traversal = self._follow_pipeline_path(
                should_stop, on_progress=publish_feedback)
            if traversal.completed:
                goal_handle.succeed()
                return self._make_result(True, traversal.distance_inspected)
            if traversal.stop_reason is _FollowStopReason.TIMED_OUT:
                goal_handle.succeed()
                return self._make_result(
                    True, traversal.distance_inspected, timed_out=True)
            if traversal.stop_reason is _FollowStopReason.DEACTIVATED:
                goal_handle.abort()
                return self._make_result(False, traversal.distance_inspected)

            goal_handle.canceled()
            return self._make_result(False, traversal.distance_inspected)
        finally:
            self._goal_executing.clear()

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start pipeline following unless action server mode is on."""
        self.get_logger().info("on_activate() is called.")
        self.abort_follow = False
        self._abort_event.clear()
        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'pipeline/get_path service is not available')
            return TransitionCallbackReturn.FAILURE

        if not use_action_server(self):
            if not self._load_pipe_path_once():
                return TransitionCallbackReturn.FAILURE

            if self.executor is None:
                self.get_logger().info('Executor is None')
                return TransitionCallbackReturn.FAILURE

            self.follow_task = self.executor.create_task(self.follow_pipeline)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node and stop pipeline following."""
        self.get_logger().info("on_deactivate() is called.")
        self._abort_event.set()
        self.abort_follow = True
        if hasattr(self, 'follow_task') and self.follow_task is not None:
            self.follow_task.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.get_logger().info('on_cleanup() is called.')
        self._abort_event.set()
        self.abort_follow = True
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_configured_entities(self) -> None:
        """Destroy entities owned by the configured lifecycle state."""
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._controller is not None:
            self._controller.destroy()
            self._controller = None
        if self.get_path_service is not None:
            self.destroy_client(self.get_path_service)
            self.get_path_service = None
        if self.pipeline_inspected_pub is not None:
            self.destroy_lifecycle_publisher(self.pipeline_inspected_pub)
            self.pipeline_inspected_pub = None
        if self.pipeline_distance_inspected_pub is not None:
            self.destroy_publisher(self.pipeline_distance_inspected_pub)
            self.pipeline_distance_inspected_pub = None
        if self.get_path_timer is not None:
            self.destroy_rate(self.get_path_timer)
            self.get_path_timer = None

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self._abort_event.set()
        self.abort_follow = True
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        return TransitionCallbackReturn.SUCCESS

    def follow_pipeline(self) -> None:
        """Follow the pipeline path (legacy lifecycle-activation mode)."""
        self.get_logger().info("Follow pipeline started")
        self.last_point = None
        self.distance_inspected = 0.0

        def should_stop() -> Optional[_FollowStopReason]:
            """Stop traversal when lifecycle deactivation requests it."""
            if self.abort_follow:
                return _FollowStopReason.LEGACY_ABORT
            return None

        def update_legacy_progress(
                distance_inspected: float, setpoint: PoseStamped) -> None:
            """Store progress in the legacy lifecycle attributes."""
            self.distance_inspected = distance_inspected
            self.last_point = setpoint

        traversal = self._follow_pipeline_path(
            should_stop, on_progress=update_legacy_progress)
        self.distance_inspected = traversal.distance_inspected
        self.last_point = traversal.last_point

    def calc_distance(
            self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Return Euclidean XY distance between two stamped poses."""
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)


def main():
    """Run the pipeline follower lifecycle node."""
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = PipelineFollowerLC('f_follow_pipeline_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
