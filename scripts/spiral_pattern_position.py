#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import PoseStamped

from mavros_wrapper.ardusub_wrapper import *


def spiral_points(i, resolution=0.52, spiral_width=1.0):
    step_idx = resolution * i
    x = np.cos(step_idx) * (spiral_width/(2*math.pi)) * step_idx
    y = np.sin(step_idx) * (spiral_width/(2*math.pi)) * step_idx
    return x, y


def main(args=None):
    rclpy.init(args=args)
    sp_publisher = BlueROVArduSubWrapper('spiral_path_publisher')
    sp_publisher.rate = sp_publisher.create_rate(2)

    thread = threading.Thread(
        target=rclpy.spin, args=(sp_publisher, ), daemon=True)
    thread.start()

    custom_mode = 'GUIDED'

    while not sp_publisher.status.armed:
        print('Robot is armed: ', sp_publisher.status.armed)
        sp_publisher.arm_motors(True)
        sp_publisher.rate.sleep()
    while sp_publisher.status.mode != custom_mode:
        print('Robot mode is : ', sp_publisher.status.mode)
        sp_publisher.set_mode(custom_mode)
        sp_publisher.rate.sleep()

    sp_publisher.declare_parameter('spiral_width', 1.0)

    i = 0
    # Ensure that UUV is at start point
    x = .0
    y = .0
    z = sp_publisher.local_pos.pose.position.z
    try:
        while rclpy.ok():
            spiral_width = sp_publisher.get_parameter(
                'spiral_width').get_parameter_value().double_value
            x, y = spiral_points(i, resolution=0.1,  spiral_width=spiral_width)
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
