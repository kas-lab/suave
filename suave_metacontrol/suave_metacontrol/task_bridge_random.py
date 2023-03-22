#!/usr/bin/env python
import random
import rclpy
import sys

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from suave.task_bridge_none import TaskBridgeNone
from system_modes_msgs.srv import ChangeMode
from system_modes_msgs.srv import GetAvailableModes


class TaskBridgeRandom(TaskBridgeNone):
    def __init__(self):
        super().__init__()

        self.declare_parameter('adaptation_period', 15)
        self.adaptation_period = self.get_parameter('adaptation_period').value

        self.generate_path_modes_cli = self.create_client(
            GetAvailableModes,
            '/f_generate_search_path/get_available_modes',
            callback_group=self.client_cb_group)

        self.follow_pipeline_modes_cli = self.create_client(
            GetAvailableModes,
            '/f_follow_pipeline/get_available_modes',
            callback_group=self.client_cb_group)

        self.available_modes_cli = {
            'f_generate_search_path': self.generate_path_modes_cli,
            'f_follow_pipeline': self.follow_pipeline_modes_cli,
        }

        self.reasoner_timer = self.create_timer(
            self.adaptation_period,
            self.reasoner_cb,
            callback_group=self.task_cb_group
        )

    def reasoner_cb(self):
        for task_name in self.current_tasks:
            function_names = self.task_functions_dict[task_name]
            for function in function_names:
                self.forward_task_request(function)

    def forward_task_request(self, function):
        modes_cli = self.available_modes_cli[function]
        mode_name = random.choice(
            self.call_service(
                modes_cli, GetAvailableModes.Request()).available_modes
            )

        cli = self.sm_cli_dict[function]
        return self.call_sysmode_change_mode(function, mode_name)


def main():
    print('Starting random task bridge node')

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridgeRandom()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
