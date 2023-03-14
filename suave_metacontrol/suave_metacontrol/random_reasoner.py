import rclpy


from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from system_modes_msgs.srv import ChangeMode, GetAvailableModes
import threading
import numpy as np
from std_msgs.msg import Bool
from std_srvs.srv import Empty


class RandomReasoner(Node):
    def __init__(self):
        super().__init__('random_reasoner')

        self.declare_parameter('adaptation_period', 15)
        self.adaptation_period = self.get_parameter('adaptation_period').value

        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        sub_cp_group = MutuallyExclusiveCallbackGroup()
        self.generate_path_changemode_cli = self.create_client(
            ChangeMode,
            '/f_generate_search_path/change_mode',
            callback_group=client_cb_group)

        self.follow_pipeline_changemode_cli = self.create_client(
            ChangeMode,
            '/f_follow_pipeline/change_mode',
            callback_group=client_cb_group)

        self.generate_path_availmodes_cli = self.create_client(
            GetAvailableModes,
            '/f_generate_search_path/get_available_modes',
            callback_group=client_cb_group)

        self.follow_pipeline_availmodes_cli = self.create_client(
            GetAvailableModes,
            '/f_follow_pipeline/get_available_modes',
            callback_group=client_cb_group)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=sub_cp_group)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=sub_cp_group)

        self.dat_req = GetAvailableModes.Request()

        self.detect_modes = self.send_request(
            self.generate_path_availmodes_cli).available_modes
        self.follow_modes = self.send_request(
            self.follow_pipeline_availmodes_cli).available_modes
        self.client = self.follow_pipeline_availmodes_cli
        self.get_logger().info(
            'Available modes detect {}'.format(str(self.detect_modes)))
        self.get_logger().info(
            'Available modes inspect {}'.format(str(self.follow_modes)))

        self.time_monitor_timer = self.create_timer(
            self.adaptation_period,
            self.change_mode_request,
            callback_group=timer_cb_group)

    def send_request(self, cli):
        while not cli.wait_for_service():
            self.get_logger().info('service not available, waiting again...')
        self.future = cli.call_async(self.dat_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data
        if self.pipeline_detected is True:
            change_req = ChangeMode.Request()
            change_req.mode_name = 'fd_unground'
            self.generate_path_changemode_cli.call(change_req)
            self.destroy_subscription(self.pipeline_detected)

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data
        if self.pipeline_detected is True:
            change_req = ChangeMode.Request()
            change_req.mode_name = 'fd_unground'
            self.follow_pipeline_changemode_cli.call(change_req)
            self.destroy_subscription(self.pipeline_inspected)

    def change_mode_request(self):
        self.get_logger().info('Start ChangeMode request')

        if not self.pipeline_detected:
            self.get_logger().info('Pipeline not detected (yet)')
            change_client = self.generate_path_changemode_cli
            modes = self.detect_modes
        elif (self.pipeline_detected and not self.pipeline_inspected):
            self.get_logger().info('Pipeline detected not inspected (yet)')
            change_client = self.follow_pipeline_changemode_cli
            modes = self.follow_modes
        else:
            self.get_logger().info('Mission done, why am I still running?')
            return

        new_mode = np.random.choice(modes)

        self.get_logger().info('Random mode {}'.format(str(new_mode)))

        change_req = ChangeMode.Request()
        change_req.mode_name = new_mode
        res = change_client.call(change_req)

        self.get_logger().info('ChangeMode success? {}'.format(str(res)))


def main(args=None):

    rclpy.init(args=args)

    random_reasoner_node = RandomReasoner()

    executor = MultiThreadedExecutor()
    executor.add_node(random_reasoner_node)

    executor.spin()

    random_reasoner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
