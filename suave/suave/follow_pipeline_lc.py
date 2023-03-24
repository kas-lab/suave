#!/usr/bin/env python
from typing import Optional

import math
import rclpy
import threading

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from suave_msgs.srv import GetPath
from suave.bluerov_gazebo import BlueROVGazebo

from std_msgs.msg import Bool
from std_msgs.msg import Float32


class PipelineFollowerLC(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.trigger_configure()
        self.abort_follow = False
        self.distance_inspected = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.ardusub = BlueROVGazebo('bluerov_pipeline_follower')
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self.get_path_timer = self.create_rate(5)
        self.get_path_service = self.create_client(
            GetPath, 'pipeline/get_path')

        self.pipeline_inspected_pub = self.create_lifecycle_publisher(
            Bool, 'pipeline/inspected', 10)

        self.pipeline_distance_inspected_pub = self.create_publisher(
            Float32, 'pipeline/distance_inspected', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'pipeline/get_path service is not available')
            return TransitionCallbackReturn.FAILURE
        if self.executor is None:
            self.get_logger().info('Executor is None')
            return TransitionCallbackReturn.FAILURE
        else:
            self.executor.create_task(self.follow_pipeline)
            self.abort_follow = False

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.abort_follow = True
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

        pipe_path = self.get_path_service.call_async(GetPath.Request())

        timer = self.ardusub.create_rate(5)  # Hz
        while not pipe_path.done():
            if self.abort_follow is True:
                return
            timer.sleep()

        last_point = None
        self.distance_inspected = 0
        for gz_pose in pipe_path.result().path.poses:
            if self.abort_follow is True:
                return
            setpoint = self.ardusub.setpoint_position_gz(
                gz_pose, fixed_altitude=True)

            count = 0
            while not self.ardusub.check_setpoint_reached_xy(setpoint, 0.5):
                if self.abort_follow is True:
                    self.distance_inspected += self.calc_distance(
                        last_point, self.ardusub.local_pos)
                    dist = Float32()
                    dist.data = self.distance_inspected
                    self.pipeline_distance_inspected_pub.publish(dist)
                    return
                if count > 10:
                    setpoint = self.ardusub.setpoint_position_gz(
                        gz_pose, fixed_altitude=True)
                count += 1
                timer.sleep()

            if last_point is not None:
                self.distance_inspected += self.calc_distance(
                    last_point, setpoint)
                dist = Float32()
                dist.data = self.distance_inspected
                self.pipeline_distance_inspected_pub.publish(dist)
            last_point = setpoint

        pipe_inspected = Bool()
        pipe_inspected.data = True
        self.pipeline_inspected_pub.publish(pipe_inspected)
        self.get_logger().info("Follow pipeline completed")

    def calc_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)


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
