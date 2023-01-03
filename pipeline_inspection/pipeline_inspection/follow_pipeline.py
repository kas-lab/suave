#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import threading

from rcl_interfaces.msg import ParameterType
from pipeline_inspection_msgs.srv import GetPath
from pipeline_inspection.bluerov_gazebo import BlueROVGazebo


def mission(ardusub):

    service_timer = ardusub.create_rate(2)

    while not ardusub.start_follow:
        pass

    pipe_path = ardusub.call_service(
        GetPath, 'pipeline_inspection/get_path', GetPath.Request())

    timer = ardusub.create_rate(5)  # Hz

    while not pipe_path.done():
        timer.sleep()

    for gz_pose in pipe_path.result().path.poses:
        setpoint = ardusub.setpoint_position_gz(gz_pose, fixed_altitude=True)
        while not ardusub.check_setpoint_reached(setpoint, delta=0.2):
            timer.sleep()

    ardusub.get_logger().info("Mission completed")


def main():
    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVGazebo("follow_pipeline_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)

    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()
