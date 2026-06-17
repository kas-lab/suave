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

import math
import time
import rclpy
import threading

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from suave_msgs.action import FollowPipeline
from suave_msgs.srv import GetPath
from suave.bluerov_gazebo import BlueROVGazebo

from std_msgs.msg import Bool
from std_msgs.msg import Float32


def spin_srv(executor):
    """Spin the executor, ignoring external shutdown."""
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


class PipelineFollowerLC(Node):
    """Lifecycle node that follows a detected pipeline path."""

    def __init__(self, node_name, **kwargs):
        """Create the pipeline follower node."""
        super().__init__(node_name, **kwargs)
        self.abort_follow = False
        self.distance_inspected = 0
        self.first_inspection = True
        self.ardusub = None
        self._action_server = None
        self.declare_parameter('use_action_server', False)
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node: create publishers, service client, and action server."""
        self.get_logger().info("on_configure() is called.")

        self.get_path_timer = self.create_rate(5)
        self.get_path_service = self.create_client(
            GetPath,
            'pipeline/get_path',
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_inspected_pub = self.create_lifecycle_publisher(
            Bool, 'pipeline/inspected', 10)

        self.pipeline_distance_inspected_pub = self.create_publisher(
            Float32, 'pipeline/distance_inspected', 10)

        if self.ardusub is None:
            self.ardusub = BlueROVGazebo('bluerov_pipeline_follower')
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self.ardusub)
            self.thread = threading.Thread(
                target=spin_srv, args=(executor, ), daemon=True)
            self.thread.start()

        self._action_server = ActionServer(
            self,
            FollowPipeline,
            'follow_pipeline',
            execute_callback=self._execute_follow_pipeline,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info("on_configure() completed")
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request):
        """Accept goals only when active and use_action_server is True."""
        if self._state_machine.current_state[1] != 'active':
            self.get_logger().warn('Goal rejected: node is not active.')
            return GoalResponse.REJECT
        if not self.get_parameter(
                'use_action_server').get_parameter_value().bool_value:
            self.get_logger().warn('Goal rejected: use_action_server is False.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Allow cancellation at any time."""
        return CancelResponse.ACCEPT

    def _execute_follow_pipeline(self, goal_handle):
        """Follow the pipeline path with timeout, feedback, and cancellation support."""
        timeout = goal_handle.request.timeout
        start_time = time.time()

        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pipeline/get_path service not available')
            result = FollowPipeline.Result()
            result.distance_inspected = 0.0
            result.success = False
            result.timed_out = False
            goal_handle.abort()
            return result

        if self.first_inspection:
            pipe_path_response = self.call_service(
                self.get_path_service, GetPath.Request())
            if pipe_path_response is None:
                result = FollowPipeline.Result()
                result.distance_inspected = 0.0
                result.success = False
                result.timed_out = False
                goal_handle.abort()
                return result
            self.pipe_path = list(pipe_path_response.path.poses)
            self.first_inspection = False

        timer = self.create_rate(0.5)
        last_point = None
        distance_inspected = 0.0

        while len(self.pipe_path) > 0:
            elapsed = time.time() - start_time
            if timeout > 0.0 and elapsed >= timeout:
                result = FollowPipeline.Result()
                result.distance_inspected = distance_inspected
                result.success = True
                result.timed_out = True
                goal_handle.succeed()
                return result

            if goal_handle.is_cancel_requested:
                result = FollowPipeline.Result()
                result.distance_inspected = distance_inspected
                result.success = False
                result.timed_out = False
                goal_handle.canceled()
                return result

            gz_pose = self.pipe_path[0]
            setpoint = self.ardusub.setpoint_position_gz(
                gz_pose, fixed_altitude=True)

            while setpoint is None:
                if goal_handle.is_cancel_requested:
                    result = FollowPipeline.Result()
                    result.distance_inspected = distance_inspected
                    result.success = False
                    result.timed_out = False
                    goal_handle.canceled()
                    return result
                timer.sleep()
                setpoint = self.ardusub.setpoint_position_gz(
                    gz_pose, fixed_altitude=True)

            count = 0
            while not self.ardusub.check_setpoint_reached_xy(setpoint, 0.5):
                if goal_handle.is_cancel_requested:
                    result = FollowPipeline.Result()
                    result.distance_inspected = distance_inspected
                    result.success = False
                    result.timed_out = False
                    goal_handle.canceled()
                    return result
                if count > 10:
                    setpoint = self.ardusub.setpoint_position_gz(
                        gz_pose, fixed_altitude=True)
                count += 1
                timer.sleep()

            self.pipe_path.pop(0)

            if last_point is not None:
                distance_inspected += self.calc_distance(last_point, setpoint)
                dist = Float32()
                dist.data = distance_inspected
                self.pipeline_distance_inspected_pub.publish(dist)

                feedback = FollowPipeline.Feedback()
                feedback.distance_inspected = distance_inspected
                goal_handle.publish_feedback(feedback)

            last_point = setpoint

        pipe_inspected = Bool()
        pipe_inspected.data = True
        self.pipeline_inspected_pub.publish(pipe_inspected)
        self.get_logger().info("Follow pipeline completed")

        result = FollowPipeline.Result()
        result.distance_inspected = distance_inspected
        result.success = True
        result.timed_out = False
        goal_handle.succeed()
        return result

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate: start follow pipeline task unless action server mode is on."""
        self.get_logger().info("on_activate() is called.")
        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'pipeline/get_path service is not available')
            return TransitionCallbackReturn.FAILURE

        use_action_server = self.get_parameter(
            'use_action_server').get_parameter_value().bool_value

        if not use_action_server:
            if self.first_inspection is True:
                self.pipe_path = self.call_service(
                    self.get_path_service, GetPath.Request())
                if self.pipe_path is None:
                    return TransitionCallbackReturn.FAILURE
                self.pipe_path = self.pipe_path.path.poses
                self.first_inspection = False

            if self.executor is None:
                self.get_logger().info('Executor is None')
                return TransitionCallbackReturn.FAILURE

            self.follow_task = self.executor.create_task(self.follow_pipeline)
            self.abort_follow = False

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node and stop pipeline following."""
        self.get_logger().info("on_deactivate() is called.")
        self.abort_follow = True
        if hasattr(self, 'follow_task') and self.follow_task is not None:
            self.follow_task.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        return TransitionCallbackReturn.SUCCESS

    def follow_pipeline(self):
        """Follow the pipeline path (legacy lifecycle-activation mode)."""
        self.get_logger().info("Follow pipeline started")

        timer = self.create_rate(0.5)  # Hz
        self.last_point = None
        self.distance_inspected = 0
        while len(self.pipe_path) > 0:
            gz_pose = self.pipe_path.pop(0)
            if self.abort_follow is True:
                return
            setpoint = self.ardusub.setpoint_position_gz(
                gz_pose, fixed_altitude=True)
            while setpoint is None:
                if self.abort_follow is True:
                    return
                timer.sleep()
                setpoint = self.ardusub.setpoint_position_gz(
                    gz_pose, fixed_altitude=True)

            count = 0
            while not self.ardusub.check_setpoint_reached_xy(setpoint, 0.5):
                if self.abort_follow is True:
                    return
                if count > 10:
                    setpoint = self.ardusub.setpoint_position_gz(
                        gz_pose, fixed_altitude=True)
                count += 1
                timer.sleep()

            if self.last_point is not None:
                self.distance_inspected += self.calc_distance(
                    self.last_point, setpoint)
                dist = Float32()
                dist.data = self.distance_inspected
                self.pipeline_distance_inspected_pub.publish(dist)
            self.last_point = setpoint

        pipe_inspected = Bool()
        pipe_inspected.data = True
        self.pipeline_inspected_pub.publish(pipe_inspected)
        self.get_logger().info("Follow pipeline completed")

    def calc_distance(self, pose1, pose2):
        """Return Euclidean XY distance between two stamped poses."""
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)

    def call_service(self, cli, request):
        """Call a ROS service synchronously, returning None on failure."""
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()


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
