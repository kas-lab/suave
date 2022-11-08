#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn  
from mavros_msgs.srv import CommandBool, SetMode

from mavros_wrapper.ardusub_wrapper import *

def check_setpoint_reached(a, x, y, delta=0.1):
    return abs(a.x - x) <= delta \
            and abs(a.y - y) <= delta

def spiral_points(i, resolution=0.1, spiral_width=1.0):
    step_idx = resolution * i
    x = np.cos(step_idx) * spiral_width * step_idx 
    y = np.sin(step_idx) * spiral_width * step_idx 
    return x,y

def main(args=None):
    rclpy.init(args=args)
    sp_publisher = BlueROVArduSubWrapper('spiral_path_publisher')
    sp_publisher.rate = sp_publisher.create_rate(2)

    thread = threading.Thread(target=rclpy.spin, args=(sp_publisher, ), daemon=True)
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

    i = 0
    # Ensure that UUV is at start point
    x = .0
    y = .0
    sp_publisher.setpoint_position_local(x,y,.0,.0,.0,.0,1.0)
    try:
        while rclpy.ok():
            if check_setpoint_reached(sp_publisher.local_pos.pose.position,x,y):
                print(i,'th iteration')
                print('x pose goal = ', x, ' y pose goal = ', y)
                x,y = spiral_points(i, resolution=0.1, spiral_width=1.0)
                sp_publisher.setpoint_position_local(x,y,.0,.0,.0,.0,1.0)
                i += 1 
            sp_publisher.rate.sleep()
    except KeyboardInterrupt:
        pass
    sp_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

