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

"""Run pipeline inspection with a configurable mission time limit."""

import sys

import rclpy

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from suave_missions.inspection_mission import InspectionMission


class MissionTimeConstrained(InspectionMission):
    """Abort and save an inspection mission after its time limit."""

    def __init__(self, node_name='time_contrained_mission'):
        """Initialize inspection behavior and the time-limit monitor."""
        super().__init__(node_name)
        if self.result_filename == 'mission_results':
            self.result_filename = 'time_mission_results'

        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value

        self.time_monitor_timer = self.create_timer(
            0.5,
            self.time_monitor_cb,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.save_mission_results_cli = self.create_client(
            Empty,
            'mission_metrics/save',
            callback_group=MutuallyExclusiveCallbackGroup())

    def time_monitor_cb(self):
        """Abort and save when elapsed time reaches the limit."""
        if self.mission_start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.mission_start_time
            if elapsed_time.to_msg().sec >= self.time_limit:
                self.abort_mission = True
                self.call_service(
                    self.save_mission_results_cli, Empty.Request())
                self.time_monitor_timer.destroy()


def main():
    """Run the time-constrained inspection mission node."""
    rclpy.init(args=sys.argv)

    mission_node = MissionTimeConstrained()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(mission_node)
    mt_executor.create_task(mission_node.perform_mission)
    mt_executor.spin()

    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
