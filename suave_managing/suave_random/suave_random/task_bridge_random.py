#!/usr/bin/env python
# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bridge tasks to randomly selected system modes."""

import random
import rclpy
import sys

from rclpy.executors import MultiThreadedExecutor
from suave.task_bridge_none import TaskBridgeNone
from system_modes_msgs.srv import GetAvailableModes


class TaskBridgeRandom(TaskBridgeNone):
    """Periodically select random available modes for active tasks."""

    def __init__(self):
        """Initialize mode clients and the periodic adaptation timer."""
        super().__init__()

        self.declare_parameter('adapt_period', 15)
        self.adaptation_period = self.get_parameter('adapt_period').value

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
        """Reconfigure every function belonging to an active task."""
        for task_name in self.current_tasks:
            function_names = self.task_functions_dict[task_name]
            for function in function_names:
                self.forward_task_request(function)

    def forward_task_request(self, function):
        """Switch a function to a randomly selected available mode."""
        modes_cli = self.available_modes_cli[function]
        mode_name = random.choice(
            self.call_service(
                modes_cli, GetAvailableModes.Request()).available_modes
            )

        return self.call_sysmode_change_mode(function, mode_name)


def main():
    """Run the random task bridge node."""
    print('Starting random task bridge node')

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridgeRandom()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
