#!/usr/bin/env python

import rclpy
import threading
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn  
from mavros_msgs.srv import CommandBool, SetMode


class MovePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
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

        self.local_pos_pub = self.create_publisher(OverrideRCIn,
                '/mavros/rc/override', 10)
        self.state = State()

    def move_publisher_callback(self, msg):
        self.local_pos_pub.publish(msg)

    def state_cb(self, msg):
        self.state = msg

    def normalized_to_pwm(self, x):
        return int((1900-1500)*x + 1500)

def main(args=None):
    rclpy.init(args=args)
    move_publisher = MovePublisher()

    thread = threading.Thread(target=rclpy.spin, args=(move_publisher, ), daemon=True)
    thread.start()

    arm_cmd = CommandBool.Request()
    arm_cmd.value = True

    mode = SetMode.Request()
    mode.custom_mode = 'ALT_HOLD'

    while not move_publisher.state.armed:
        print('Robot is armed: ', move_publisher.state.armed)
        move_publisher.arm_client.call_async(arm_cmd)
        move_publisher.rate.sleep()
    while move_publisher.state.mode != mode.custom_mode:
        print('Robot mode is : ', move_publisher.state.mode)
        move_publisher.mode_client.call_async(mode)
        move_publisher.rate.sleep()

    msg = OverrideRCIn()
    start_time = move_publisher.get_clock().now().to_msg().sec 
    while move_publisher.get_clock().now().to_msg().sec-start_time<5:
        # print(move_publisher.get_clock().now().to_msg().sec)
        movement = np.zeros(18, dtype=np.uint16)
        movement[4] = 1
        msg.channels = movement
        move_publisher.move_publisher_callback(msg)
        move_publisher.rate.sleep()

    print('moved 5 seconds')
    
    i = 0
    start_time = move_publisher.get_clock().now().to_msg().sec 
    while move_publisher.get_clock().now().to_msg().sec-start_time<5:
        movement = np.zeros(18, dtype=np.uint16)
        movement[3] = move_publisher.normalized_to_pwm(0.2) 
        msg.channels = movement
        move_publisher.move_publisher_callback(msg)
        i += 1
    print('turned for 5 secs')
    try:
        while rclpy.ok():
            move_publisher.rate.sleep()
    except KeyboardInterrupt:
        pass
    move_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

