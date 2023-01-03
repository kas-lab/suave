#!/usr/bin/env python
import sys

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from pipeline_inspection.pipeline_node import PipelineNode


class PipelineNodeMC(PipelineNode):
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

    def compare_poses(self, pose1, pose2, delta=1.):
        result = False
        if self.water_visibility is not None:
            result = abs(pose1.position.x - pose2.position.x) <= delta \
                and abs(pose1.position.y - pose2.position.y) <= delta \
                and abs(pose1.position.z - pose2.position.z) \
                <= self.water_visibility
        return result


def main():
    rclpy.init(args=sys.argv)

    detect_pipeline_node = PipelineNodeMC()
    rclpy.spin(detect_pipeline_node)

    rclpy.shutdown()
