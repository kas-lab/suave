import rclpy
import threading
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget  
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

        # self.local_pos_pub = self.create_publisher(PoseStamped,
                # '/mavros/setpoint_position/local', 10)

        self.local_vel_pub = self.create_publisher(Twist,
                '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
                
        # self.raw_vel_pub = self.create_publisher(PositionTarget,
                # '/mavros/setpoint_raw/local', 10)

        self.state = State()

    # def move_publisher_callback(self, msg):
        # self.local_pos_pub.publish(msg)

    def move_publisher_callback(self, vel):
        print('forward vel=', vel.linear.x, ' angular vel=',
                vel.angular.z)
        self.local_vel_pub.publish(vel)

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
    mode.custom_mode = 'GUIDED'

    while not move_publisher.state.armed:
        print('Robot is armed: ', move_publisher.state.armed)
        move_publisher.arm_client.call_async(arm_cmd)
        move_publisher.rate.sleep()
    while move_publisher.state.mode != mode.custom_mode:
        print('Robot mode is : ', move_publisher.state.mode)
        move_publisher.mode_client.call_async(mode)
        move_publisher.rate.sleep()

    vel = Twist()
    forward_vel = 5.0
    angular_vel = 0.0
    vel.linear.x = forward_vel
    vel.linear.y = 0.0
    vel.linear.z = 0.0
    vel.angular.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z = angular_vel
    # vel.twist.angular.z = angular_vel

    print('moving with forward_vel={} and angular_vel={}'.format(forward_vel,
        angular_vel))

    i=0
    try:
        while rclpy.ok():
            print(i,'th print')
            move_publisher.move_publisher_callback(vel)
            move_publisher.rate.sleep()
            i+=1
    except KeyboardInterrupt:
        pass
    move_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

