#!/usr/bin/env python
import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from suave.pipeline_detection import PipelineDetection


class PipelineDetectionWV(PipelineDetection):
    def __init__(self):
        super().__init__()

        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_cb, 10)

        self.water_visibility = None

    def diagnostics_cb(self, msg):
        for status in msg.status:
            if status.message == "QA status":
                for value in status.values:
                    if value.key == "water_visibility":
                        self.water_visibility = float(value.value)

    def compare_poses(self, bluerov_pose, pipe_pose):
        result = False
        if self.water_visibility is not None:
            result = super().compare_poses(bluerov_pose, pipe_pose)
            result = result and \
                abs(bluerov_pose.position.z - pipe_pose.position.z) \
                <= self.water_visibility
        return result


def main():
    rclpy.init(args=sys.argv)

    detect_pipeline_node = PipelineDetectionWV()
    rclpy.spin(detect_pipeline_node)

    rclpy.shutdown()
