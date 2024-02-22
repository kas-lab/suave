#!/usr/bin/env python
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from suave.pipeline_detection import PipelineDetection
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class PipelineDetectionWV(PipelineDetection):
    def __init__(self):
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


def main(args=None):
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
