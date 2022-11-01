import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped  
from mavros_msgs.msg import State  
from mavros_msgs.srv import CommandBool, SetMode

class MovePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.rate = self.create_rate(2)
        self.state_sub = self.create_subscription(State, '/mavros/state',
                self.state_cb, 10)
        self.state_sub
        print(self.state_sub)
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
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.move_publisher_callback) 
        # self.i = 0
        self.state = State()

    def move_publisher_callback(self, msg):
        self.local_pos_pub.publish(msg)
        self.get_logger().info('Publishing point x: "%s"' % msg.pose.position.x)

    def state_cb(self, msg):
        print('Entered state cb')
        self.state = msg

def main(args=None):
    rclpy.init(args=args)

    move_publisher = MovePublisher()

    arm_cmd = CommandBool.Request()
    arm_cmd.value = True

    # rclpy.spin(move_publisher)
    mode = SetMode.Request()
    mode.custom_mode = 'GUIDED'
    i = 0
    print('Im here')
    print('Robot is connected: ', move_publisher.state.connected)
    while rclpy.ok():
        print('Robot is connected: ', move_publisher.state.connected)
        while not move_publisher.state.armed:
            print('Robot is armed: ', move_publisher.state.armed)
            move_publisher.arm_client.call_async(arm_cmd)
            rclpy.spin_once(move_publisher)
            print('after arm client call')
        print('after arming')
        while move_publisher.state.mode != 'GUIDED':
            print('Robot mode is : ', move_publisher.state.mode)
            move_publisher.mode_client.call_async(mode)
            rclpy.spin_once(move_publisher)
        if i%20 == 0:
            print(i/20)
            msg = PoseStamped()
            msg.pose.position.x = i/20
            msg.pose.position.y = 0.0 
            msg.pose.position.z = 0.0 
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0 
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            move_publisher.move_publisher_callback(msg)
        i += 1

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        rclpy.spin_once(move_publisher)
        print('after spin_once')
    move_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

