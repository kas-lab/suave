#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn  
from mavros_msgs.srv import CommandBool, SetMode

def spiral_points(spiral_iteration, old_x, old_y, spiral_width):
    spiral_width = 0.3
    new_x = old_x + np.cos(spiral_width * spiral_iteration) * 0.02 * spiral_iteration 
    new_y = old_y + np.sin(spiral_width * spiral_iteration) * 0.02 * spiral_iteration 

    spiral_iteration += 1.

    return new_x, new_y, spiral_iteration 

class MovePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        self.rate = self.create_rate(2)

        self.state_sub = self.create_subscription(State, '/mavros/state',
                self.state_cb, 10)
        self.state_sub

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            print('Waiting for arm service')
            self.get_logger().info('service not available, waiting again...')

        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            print('Waiting for mode service')
            self.get_logger().info('service not available, waiting again...')

        self.local_pos_pub = self.create_publisher(PoseStamped,
                '/mavros/setpoint_position/local', 10)

        self.state = State()

    def pose_publisher_cb(self, point):
        print('x position=', point.pose.position.x, ' y position=',
                point.pose.position.y)
        self.local_pos_pub.publish(point)

    def state_cb(self, msg):
        self.state = msg

def main(args=None):
    rclpy.init(args=args)
    move_publisher = MovePublisher()

    thread = threading.Thread(target=rclpy.spin, args=(move_publisher, ), daemon=True)
    thread.start()

    arm_cmd = CommandBool.Request()
    arm_cmd.value = True

    mode = SetMode.Request()
    mode.custom_mode = 'GUIDED'

    while not move_publisher.state.armed:
        print('Robot is armed: ', move_publisher.state.armed)
        move_publisher.arm_client.call_async(arm_cmd)
        move_publisher.rate.sleep()
    while move_publisher.state.mode != mode.custom_mode:
        print('Robot mode is : ', move_publisher.state.mode)
        move_publisher.mode_client.call_async(mode)
        move_publisher.rate.sleep()

    # Initialize variables
    spiral_iteration = 0
    x = 0
    y = 0
    i = 0
    point = PoseStamped()
    try:
        while rclpy.ok():
            if i%4==0:
                print(spiral_iteration,'th iteration')
                x,y,spiral_iteration = spiral_points(spiral_iteration, x, y, 1.0)
                point.pose.position.x = x
                point.pose.position.y = y
                move_publisher.pose_publisher_cb(point)
            move_publisher.rate.sleep()
            i += 1 
    except KeyboardInterrupt:
        pass
    move_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

