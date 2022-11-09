#!/usr/bin/env python
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from pipeline_inspection.srv import GetPath


def compare_poses(pose1, pose2, delta=0.1):
    return abs(pose1.position.x - pose2.position.x) <= delta \
            and abs(pose1.position.y - pose2.position.y) <= delta


class PipelineNode(Node):

    def __init__(self):
        super().__init__('pipeline')
        self.min_pipes_pipeline_pose_subscription = self.create_subscription(
            PoseArray,
            '/model/min_pipes_pipeline/pose',
            self.pipeline_pose_cb,
            10
        )

        self.bluerov2_pose = self.create_subscription(
            Pose,
            '/model/bluerov2/pose',
            self.detect_pipeline_cb,
            10
        )

        self.detect_pipeline_pub = self.create_publisher(
            Bool, 'pipeline/detected', 10)

        self.pipes_pose_array = PoseArray()
        self.interpolation_number = 10
        self.interpolated_path = PoseArray()

        self.get_interpolated_path_srv = self.create_service(
            GetPath,
            'pipeline_inspection/get_path',
            self.get_interpolated_path_cb)

    def pipeline_pose_cb(self, msg):
        self.pipes_pose_array = msg
        self.destroy_subscription(self.min_pipes_pipeline_pose_subscription)
        self.calculate_interpolated_path()

    def interpolate_line_by_points(self, x1, y1, x2, y2, n):
        line_slope = (y2-y1)/(x2-x1)
        b = y1 - line_slope*x1
        points = []
        xn = x1
        delta_x = (x2-x1)/n
        for i in range(n):
            xn = xn + delta_x
            yn = line_slope*xn + b
            pose = Pose()
            pose.position.x = xn
            pose.position.y = yn
            points.append(pose)
        return points

    def calculate_interpolated_path(self):
        for pose_index in range(len(self.pipes_pose_array.poses)-1):
            pose_1 = self.pipes_pose_array.poses[pose_index]
            pose_2 = self.pipes_pose_array.poses[pose_index+1]
            self.interpolated_path.poses.extend(
                self.interpolate_line_by_points(
                    pose_1.position.x,
                    pose_1.position.y,
                    pose_2.position.x,
                    pose_2.position.y,
                    self.interpolation_number)
            )

    def get_interpolated_path_cb(self, req, response):
        response.path = self.interpolated_path
        return response

    def detect_pipeline_cb(self, bluerov_pose):
        for pipe_pose in self.interpolated_path.poses:
            if compare_poses(bluerov_pose, pipe_pose):
                pipe_detected = Bool()
                pipe_detected.data = True
                self.detect_pipeline_pub.publish(pipe_detected)
                break


if __name__ == '__main__':
    print("Starting detect pipeline node")

    rclpy.init(args=sys.argv)

    detect_pipeline_node = PipelineNode()
    rclpy.spin(detect_pipeline_node)

    rclpy.shutdown()
