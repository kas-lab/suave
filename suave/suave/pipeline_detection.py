#!/usr/bin/env python
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from suave_msgs.srv import GetPath

class PipelineDetection(Node):

    def __init__(self):
        super().__init__('detect_pipeline')
        self.min_pipes_pipeline_pose_subscription = self.create_subscription(
            PoseArray,
            '/model/min_pipes_pipeline/pose',
            self.pipeline_pose_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.first_detection = True
        self.bluerov2_pose_sub = self.create_subscription(
            Pose,
            '/model/bluerov2/pose',
            self.detect_pipeline_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.detect_pipeline_pub = self.create_publisher(
            Bool, 'pipeline/detected', 10)

        self.pipes_pose_array = PoseArray()
        self.interpolation_number = 20
        self.interpolated_path = PoseArray()

        self.get_interpolated_path_srv = self.create_service(
            GetPath,
            'pipeline/get_path',
            self.get_interpolated_path_cb)

        self.sorted_path = PoseArray()

        # TODO: ROS param?
        self.camera_fov = math.pi/3

    def pipeline_pose_cb(self, msg):
        self.pipes_pose_array = msg
        self.destroy_subscription(self.min_pipes_pipeline_pose_subscription)
        self.calculate_interpolated_path()

    def interpolate_line_by_points(self, pose1, pose2, n):
        x1 = pose1.position.x
        y1 = pose1.position.y
        x2 = pose2.position.x
        y2 = pose2.position.y
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
            # pose.position.z = pose1.position.z + 0.75
            pose.position.z = pose1.position.z
            points.append(pose)
        return points

    def calculate_interpolated_path(self):
        for pose_index in range(len(self.pipes_pose_array.poses)-1):
            pose1 = self.pipes_pose_array.poses[pose_index]
            pose2 = self.pipes_pose_array.poses[pose_index+1]
            self.interpolated_path.poses.extend(
                self.interpolate_line_by_points(
                    pose1, pose2, self.interpolation_number))

    def get_interpolated_path_cb(self, req, response):
        response.path = self.sorted_path
        return response

    def compare_poses(self, bluerov_pose, pipe_pose):
        altitude = abs(bluerov_pose.position.z - pipe_pose.position.z)
        delta = altitude * math.tan(self.camera_fov/2)
        return abs(bluerov_pose.position.x - pipe_pose.position.x) <= delta \
            and abs(bluerov_pose.position.y - pipe_pose.position.y) <= delta

    def detect_pipeline_cb(self, bluerov_pose):
        for i in range(len(self.interpolated_path.poses)):
            if self.compare_poses(bluerov_pose,
               self.interpolated_path.poses[i]):
                pipe_detected = Bool()
                pipe_detected.data = True
                self.detect_pipeline_pub.publish(pipe_detected)
                if self.first_detection:
                    self.sort_pipe_path(i)
                    self.first_detection = False
                    self.destroy_subscription(self.bluerov2_pose_sub)
                return

    def sort_pipe_path(self, index):
        delta = 1
        self.sorted_path = PoseArray()
        self.sorted_path.poses.extend(
            self.interpolated_path.poses[index::delta])
        self.sorted_path.poses.extend(
            reversed(self.interpolated_path.poses[::delta]))


def main(args=None):
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        lc_node = PipelineDetection()
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
