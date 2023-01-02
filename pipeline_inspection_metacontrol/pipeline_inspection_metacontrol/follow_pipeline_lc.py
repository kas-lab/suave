#!/usr/bin/env python
from typing import Optional

import rclpy
import threading

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from pipeline_inspection_msgs.srv import GetPath
from pipeline_inspection.bluerov_gazebo import BlueROVGazebo

import std_msgs.msg


class PipelineFollower(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.ardusub = BlueROVGazebo()

        # I am not sure this is needed, use multi-threaded executor instead?
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self.get_path_timer = self.create_rate(5)
        self.get_path_service = self.create_client(
            GetPath, 'pipeline_inspection/get_path')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")
        if self.executor is not None:
            self.executor.create_task(self.mission)
        else:
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.mission_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.mission_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def mission(self):
        self.get_logger().info("Mission started")

        while not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                 'pipeline_inspection/get_path service is not available')
            self.get_path_timer.sleep()
        pipe_path = self.get_path_service.call_async(GetPath.Request())

        timer = self.create_rate(5)  # Hz
        while not pipe_path.done():
            timer.sleep()

        for gz_pose in pipe_path.result().path.poses:
            setpoint = self.ardusub.setpoint_position_gz(
                gz_pose, fixed_altitude=True)
            while not self.ardusub.check_setpoint_reached(setpoint, 0.2):
                timer.sleep()

        self.get_logger().info("Mission completed")
        self.trigger_deactivate()


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = PipelineFollower('pipeline_follower')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
