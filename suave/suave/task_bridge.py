#!/usr/bin/env python
import sys
import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from suave_msgs.srv import Task
from system_modes_msgs.srv import ChangeMode


class TaskBridge(Node):
    def __init__(self):
        super().__init__('task_bridge')
        self.declare_parameter('f_generate_search_path_mode', 'fd_spiral_low')
        self.declare_parameter('f_follow_pipeline_mode', 'fd_follow_pipeline')

        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.generate_path_sm_cli = self.create_client(
            ChangeMode,
            '/f_generate_search_path/change_mode',
            callback_group=client_cb_group)

        self.follow_pipeline_sm_cli = self.create_client(
            ChangeMode,
            '/f_follow_pipeline/change_mode',
            callback_group=client_cb_group)

        self.sm_cli_dict = {
            'generate_search_path': self.generate_path_sm_cli,
            'follow_pipeline': self.follow_pipeline_sm_cli,
        }

        self.task_request_service = self.create_service(
            Task,
            'task/request',
            self.task_request_cb)

    def task_request_cb(self, req, response):
        try:
            mode_name = self.get_parameter(
                'f_' + req.task_name + '_mode').value
        except Exception as e:
            self.get_logger().error(
                str(e) + '. Probably requested wrong task name.')
            response = Task.Response()
            response.success = False
            return response

        cli = self.sm_cli_dict[req.task_name]
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        mode_req = ChangeMode.Request()
        mode_req.mode_name = mode_name
        future = cli.call_async(mode_req)
        timer = self.create_rate(5)
        rclpy.spin_until_future_complete(self, future)
        response = Task.Response()
        response.success = future.result().success
        return response


def main():
    print("Starting task bridge node")

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridge()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
