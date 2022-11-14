#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Bool

from mavros_wrapper.ardusub_wrapper import *

class MissionNode(BlueROVArduSubWrapper):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)

        self.pipeline_detected = False
        
        self.start_search_pub = self.create_publisher(Bool,
                'start_search', 10)
        self.start_follow_pipe_pub = self.create_publisher(Bool,
                'start_follow_pipe', 10)

        self.pipeline_detected_sub = self.create_subscription(Bool,
                'pipeline/detected', self.pipeline_detected_cb, 10)

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data


def mission(args=None):
    rclpy.init(args=args)
    mission_node = MissionNode() 
    mission_node.rate = mission_node.create_rate(2)

    thread = threading.Thread(target=rclpy.spin, args=(mission_node, ), daemon=True)
    thread.start()

    custom_mode = 'GUIDED'

    while not mission_node.status.armed:
        print('Robot is armed: ', mission_node.status.armed)
        mission_node.arm_motors(True)
        mission_node.rate.sleep()
    while mission_node.status.mode != custom_mode:
        print('Robot mode is : ', mission_node.status.mode)
        mission_node.set_mode(custom_mode)
        mission_node.rate.sleep()

    print('start_search')
    start_search_msg = Bool()
    start_search_msg.data = True

    while not mission_node.pipeline_detected:
        mission_node.start_search_pub.publish(start_search_msg)
        print('in while loop')
        pass

    start_follow_msg.data = False
    mission_node.start_search_pub.publish(start_search_msg)

    start_follow_msg = Bool()
    start_follow_msg.data = True
    mission_node.start_follow_pipe_pub.publish(start_follow_msg)

    while start_follow_msg.data:
        print('after start_follow_msg published')

    mission_node.rate.sleep()

    
if __name__=='__main__':
    mission()
