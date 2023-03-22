#!/usr/bin/env python
import sys
import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from suave.task_bridge import TaskBridge
from system_modes_msgs.srv import ChangeMode


class TaskBridgeNone(TaskBridge):
    def __init__(self):
        super().__init__()
        self.declare_parameter('f_generate_search_path_mode', 'fd_spiral_low')
        self.declare_parameter('f_follow_pipeline_mode', 'fd_follow_pipeline')

        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.generate_path_sm_cli = self.create_client(
            ChangeMode,
            '/f_generate_search_path/change_mode',
            callback_group=self.client_cb_group)

        self.follow_pipeline_sm_cli = self.create_client(
            ChangeMode,
            '/f_follow_pipeline/change_mode',
            callback_group=self.client_cb_group)

        self.sm_cli_dict = {
            'f_generate_search_path': self.generate_path_sm_cli,
            'f_follow_pipeline': self.follow_pipeline_sm_cli,
        }

        self.task_functions_dict = {
            'search_pipeline': ['f_generate_search_path'],
            'inspect_pipeline': ['f_follow_pipeline'],
        }

    def forward_task_request(self, function):
        mode_name = self.get_parameter(function + '_mode').value
        return self.call_sysmode_change_mode(function, mode_name)

    def forward_task_cancel_request(self, function):
        mode_name = 'fd_unground'
        return self.call_sysmode_change_mode(function, mode_name)

    def call_sysmode_change_mode(self, function, mode_name):
        mode_req = ChangeMode.Request()
        mode_req.mode_name = mode_name
        cli = self.sm_cli_dict[function]
        response = self.call_service(cli, mode_req)
        return response.success


def main():
    print('Starting task bridge node')

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridgeNone()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
