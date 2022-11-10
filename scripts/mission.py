#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Bool

from mavros_wrapper.ardusub_wrapper import *

def mission(args=None):
    rclpy.init(args=args)
    mission_node = BlueROVArduSubWrapper('mission_node') 
    mission_node.rate = mission_node.create_rate(2)

    thread = threading.Thread(target=rclpy.spin, args=(mission_node, ), daemon=True)
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

    mission_node.start_search_pub = mission_node.create_publisher(Bool,
            'start_search', 10)
    start_search_msg = Bool()
    start_search_msg.data = True
    mission_node.start_search_pub.publish(start_search_msg)


    mission_node.start_follow_pipe_pub = mission_node.create_publisher(Bool,
            'start_follow_pipe', 10)
    start_follow_msg = Bool()
    start_follow_msg.data = True
    mission_node.start_follow_pipe_pub.publish(start_follow_msg)

    
if __name__=='__main__':
    mission()
