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

"""Bridge mission tasks directly to system-mode changes."""

import sys
import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from suave.task_bridge import TaskBridge
from system_modes_msgs.srv import ChangeMode


class TaskBridgeNone(TaskBridge):
    """Execute mission tasks without an adaptation manager."""

    def __init__(self):
        """Initialize mode parameters, clients, and task mappings."""
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
        """Activate the configured system mode for a mission function."""
        mode_name = self.get_parameter(function + '_mode').value
        return self.call_sysmode_change_mode(function, mode_name)

    def forward_task_cancel_request(self, function):
        """Cancel a mission function by switching it to the unground mode."""
        mode_name = 'fd_unground'
        return self.call_sysmode_change_mode(function, mode_name)

    def call_sysmode_change_mode(self, function, mode_name):
        """Request a system-mode change for the specified function."""
        mode_req = ChangeMode.Request()
        mode_req.mode_name = mode_name
        cli = self.sm_cli_dict[function]
        response = self.call_service(cli, mode_req)
        return response.success


def main():
    """Run the direct task bridge node."""
    print('Starting task bridge node')

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridgeNone()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
