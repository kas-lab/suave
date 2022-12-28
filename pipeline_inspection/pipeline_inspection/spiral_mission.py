#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool

from mavros_wrapper.ardusub_wrapper import *


def spiral_points(i, resolution=0.1, spiral_width=1.0):
    step_idx = resolution * i
    x = np.cos(step_idx) * spiral_width * step_idx
    y = np.sin(step_idx) * spiral_width * step_idx
    return x, y


class SpiralPub(BlueROVArduSubWrapper):
    def __init__(self, node_name='spiral_path_pub'):
        super().__init__(node_name)

        self.create_subscription(
                Bool, 'start_search', self.search_cb, 10)

        self.search_pipeline = False

    def search_cb(self, msg):
        self.search_pipeline = msg.data


def main(args=None):
    rclpy.init(args=args)
    sp_publisher = SpiralPub()
    sp_publisher.rate = sp_publisher.create_rate(2)

    thread = threading.Thread(
        target=rclpy.spin, args=(sp_publisher, ), daemon=True)
    thread.start()

    sp_publisher.declare_parameter('spiral_width', 1.0)

    i = 0
    # Ensure that UUV is at start point
    x = .0
    y = .0
    z = sp_publisher.local_pos.pose.position.z
    try:
        while rclpy.ok():
            if sp_publisher.search_pipeline:
                spiral_width = sp_publisher.get_parameter(
                        'spiral_width').get_parameter_value().double_value
                x, y = spiral_points(
                    i, resolution=0.1,  spiral_width=spiral_width)
                # x,y = [0.,-3.]
                i += 1
                sp_publisher.setpoint_position_local(x, y, z)
                while not sp_publisher.check_setpoint_reached(
                        sp_publisher.pose_stamped(x, y, z), delta=0.2):
                    sp_publisher.rate.sleep()
    except KeyboardInterrupt:
        pass
    sp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
