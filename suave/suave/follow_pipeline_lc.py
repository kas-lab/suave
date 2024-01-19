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

import math
import rclpy
import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from suave_msgs.srv import GetPath
from suave.bluerov_gazebo import BlueROVGazebo

from std_msgs.msg import Bool
from std_msgs.msg import Float32


def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


class PipelineFollowerLC(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.abort_follow = False
        self.distance_inspected = 0
        self.first_inspection = True
        self.ardusub = None
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
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
        self.get_logger().info("on_configure() completed")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'pipeline/get_path service is not available')
            return TransitionCallbackReturn.FAILURE

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
        self.get_logger().info("on_deactivate() is called.")
        self.abort_follow = True
        self.follow_task.cancel()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.follow_pipeline_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.follow_pipeline_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def follow_pipeline(self):
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
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)

    def call_service(self, cli, request):
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
