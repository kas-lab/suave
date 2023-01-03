#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import threading

from mavros_wrapper.ardusub_wrapper import *

from rcl_interfaces.msg import ParameterType


def compare_positions(a, b, delta=0.5):
    return abs(a.x - b['x']) <= delta \
            and abs(a.y - b['y']) <= delta \
            and abs(a.z - b['z']) <= delta


def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    custom_mode = "GUIDED"
    while ardusub.status.mode != custom_mode:
        ardusub.set_mode(custom_mode)
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed is False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    delta = 50.0
    desired_waypoints = [
        {'x': -delta, 'y': .0, 'z': -.5},
        {'x': -delta, 'y': delta, 'z': -.5},
        {'x': .0, 'y': delta, 'z': -.5},
        {'x': .0, 'y': .0, 'z': -.5}
    ]
    timer = ardusub.create_rate(5)  # Hz

    for waypoint in desired_waypoints:
        ardusub.setpoint_position_local(**waypoint)
        while not compare_positions(ardusub.local_pos.pose.position, waypoint):
            timer.sleep()

    print("Mission completed")


def main():
    print("Starting Bluerov agent node")

    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("follow_waypoint_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)

    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()
