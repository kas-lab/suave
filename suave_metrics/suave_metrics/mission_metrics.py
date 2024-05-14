# Copyright 2024 Gustavo Rezende Silva
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
"""module used to save metrics."""
import csv
import os
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Pose
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_srvs.srv import Empty


class MissionMetrics(Node):
    """ROS node used to save mission metrics.

    ROS params: `result_path`, `result_filename`, `adaptation_manager`,
    `mission_name`.

    """

    def __init__(self, node_name: str = 'suave_metrics'):
        super().__init__(node_name)

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')
        self.declare_parameter('adaptation_manager', 'none')
        self.declare_parameter('mission_name', 'inspection')
        # self.declare_parameter('metrics_header', [''])

        #: path where results must be saved
        self.result_path = self.get_parameter('result_path').value
        #: file name where results must be saved
        self.result_filename = self.get_parameter('result_filename').value
        #: adaptation manager name, e.g., none, random, metacontrol
        self.adaptation_manager = self.get_parameter(
            'adaptation_manager').value

        self.mission_name = self.get_parameter('mission_name').value
        self.mission_name += ' ' + self.adaptation_manager

        self.metrics_header = [
            'mission name',
            'datetime',
            'initial pos (x,y)',
            'mission duration (s)',
            'pipeline found',
            'time searching pipeline (s)',
            'distance inspected (m)']

        self.mission_start_time = None
        self.pipeline_detected_time = None

        # TODO: get this automatically
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.first_gz_pose = True

        self.distance_inspected = 0.0

        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.status_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.gazebo_pos_sub = self.create_subscription(
            Pose,
            'model/bluerov2/pose',
            self.gazebo_pos_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.save_mission_results_srv = self.create_service(
            Empty,
            'mission_metrics/save',
            self.save_mission_results_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def status_cb(self, msg):
        if msg.mode == 'GUIDED':
            self.mission_start_time = self.get_clock().now()
            self.destroy_subscription(self.state_sub)

    def pipeline_detected_cb(self, msg):
        if msg.data is True:
            self.pipeline_detected_time = self.get_clock().now()
            self.destroy_subscription(self.pipeline_detected_sub)

    def pipeline_inspected_cb(self, msg):
        if msg.data is True:
            self.save_mission_results()
            self.destroy_subscription(self.pipeline_inspected_sub)

    def distance_inspected_cb(self, msg):
        self.distance_inspected = msg.data

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gz_pose is True:
            self.first_gz_pose = False
            self.initial_x = msg.position.x
            self.initial_y = msg.position.y
            self.destroy_subscription(self.gazebo_pos_sub)

    def save_mission_results_cb(
         self, req: Empty.Request, res: Empty.Response) -> None:
        self.save_mission_results()
        self.destroy_subscription(self.gazebo_pos_sub)
        self.destroy_subscription(self.pipeline_detected_sub)
        self.destroy_subscription(self.pipeline_inspected_sub)
        self.destroy_subscription(self.pipeline_distance_inspected_sub)
        return res

    def save_mission_results(self) -> None:
        """Process mission data and save it into a .csv file."""
        mission_time_delta = \
            self.get_clock().now() - self.mission_start_time

        pipeline_detected = False
        detection_time_delta = mission_time_delta.to_msg().sec
        if self.pipeline_detected_time is not None:
            detection_time_delta = \
                self.pipeline_detected_time - self.mission_start_time
            detection_time_delta = detection_time_delta.to_msg().sec
            pipeline_detected = True

        self.get_logger().info(
            'Time elapsed to detect pipeline: {} seconds'.format(
                detection_time_delta))
        self.get_logger().info(
            'Distance inspected: {} m'.format(
                self.distance_inspected))
        self.get_logger().info(
            'Time elapsed to complete mission: {} seconds'.format(
                mission_time_delta.to_msg().sec))

        mission_data = [
            self.mission_name,
            datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
            '({0}, {1})'.format(
                round(self.initial_x, 2), round(self.initial_y, 2)),
            mission_time_delta.to_msg().sec,
            pipeline_detected,
            detection_time_delta,
            self.distance_inspected
        ]

        self.save_metrics(mission_data)
        # TODO: make this a parameter
        os.system("touch ~/suave_ws/mission.done")

    def save_metrics(self, data: list[str | float | int]) -> None:
        """Save data into a .csv file.

        Create folder if `result_path` folder does not exist.
        Create file if `result_file` does not exist and insert header.
        Append data if `result_file` exist.

        :param data: array with data to be saved
        """
        result_path = Path(self.result_path).expanduser()

        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)

        result_file = result_path / (self.result_filename + '.csv')
        if result_file.is_file() is False:
            result_file.touch()
            self.append_csv(result_file, self.metrics_header)

        self.append_csv(result_file, data)

    def append_csv(self, file_path: str,
                   data: list[str | float | int]) -> None:
        """Append array to .csv file.

        :param file_path: file path
        :param data: data to be saved
        """
        with open(file_path, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data)


def main():
    rclpy.init(args=sys.argv)

    metrics_node = MissionMetrics()

    mt_executor = rclpy.executors.MultiThreadedExecutor()
    mt_executor.add_node(metrics_node)
    mt_executor.spin()

    metrics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
