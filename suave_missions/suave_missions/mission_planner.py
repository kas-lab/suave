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

"""Provide common task orchestration for SUAVE missions."""

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from suave_msgs.srv import Task


class MissionPlanner(Node):
    """Base ROS node for requesting and monitoring mission tasks."""

    def __init__(self, node_name='mission_node'):
        """Initialize task clients, result parameters, and mission state."""
        super().__init__(node_name)

        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.task_request_service = self.create_client(
            Task, 'task/request', callback_group=self.cb_group)
        self.task_cancel_service = self.create_client(
            Task, 'task/cancel', callback_group=self.cb_group)

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')

        self.result_path = self.get_parameter('result_path').value
        self.result_filename = self.get_parameter('result_filename').value

        self.metrics_header = ['mission_name', 'datetime', 'metric']

        self.mission_start_time = None
        self.abort_mission = False

    def request_task(self, task_name):
        """Request execution of a named mission task."""
        req = Task.Request()
        req.task_name = task_name
        return self.call_service(self.task_request_service, req)

    def cancel_task(self, task_name):
        """Request cancellation of a named mission task."""
        req = Task.Request()
        req.task_name = task_name
        return self.call_service(self.task_cancel_service, req)

    def call_service(self, cli, request):
        """Call a mission service with a five-second timeout."""
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()

    def perform_mission(self):
        """Report that subclasses must define mission behavior."""
        self.get_logger().warning('No mission defined!!!')

    def perform_task(self, task_name, condition):
        """Run a task until its completion condition or mission abortion."""
        self.task_timer = self.create_rate(10)
        task_status = 'completed'
        if self.abort_mission is False:
            self.get_logger().info('Starting {} task'.format(task_name))
            self.request_task(task_name)
            while condition() is not True:
                if self.abort_mission is True:
                    task_status = 'aborted'
                    break
                self.task_timer.sleep()
        else:
            task_status = 'aborted'
        self.get_logger().info('Task {0} {1}'.format(task_name, task_status))
        self.cancel_task(task_name)
