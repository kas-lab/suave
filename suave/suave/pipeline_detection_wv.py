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

"""Detect pipelines while accounting for observed water visibility."""

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from suave.pipeline_detection import PipelineDetection
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class PipelineDetectionWV(PipelineDetection):
    """Restrict geometric pipeline detection by water visibility distance."""

    def __init__(self):
        """Initialize pipeline detection and subscribe to diagnostics."""
        super().__init__()

        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.water_visibility = None

    def diagnostics_cb(self, msg):
        """Update the current water visibility from diagnostic values."""
        for status in msg.status:
            if status.message == "QA status":
                for value in status.values:
                    if value.key == "water_visibility":
                        self.water_visibility = float(value.value)

    def compare_poses(self, bluerov_pose, pipe_pose):
        """Return whether a pipeline pose is visible in geometry and water."""
        result = False
        if self.water_visibility is not None:
            result = super().compare_poses(bluerov_pose, pipe_pose)
            result = result and \
                abs(bluerov_pose.position.z - pipe_pose.position.z) \
                <= self.water_visibility
        return result


def main(args=None):
    """Run the water-visibility-aware pipeline detection node."""
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        lc_node = PipelineDetectionWV()
        executor.add_node(lc_node)
        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            executor.shutdown()
            lc_node.destroy_node()
        finally:
            executor.shutdown()
            lc_node.destroy_node()
    finally:
        rclpy.shutdown()
