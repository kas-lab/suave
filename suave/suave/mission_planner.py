import csv
import rclpy
from pathlib import Path
from datetime import datetime
from rclpy.node import Node
from suave.bluerov_gazebo import BlueROVGazebo
from std_msgs.msg import Bool


class MissionPlanner(BlueROVGazebo):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self.pipeline_detected_cb, 10)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool, 'pipeline/inspected', self.pipeline_inspected_cb, 10)

        self.mros_action_client = ActionClient(
            self, ControlQos, 'mros_objective')

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')

        self.result_path = self.get_parameter('result_path').value

        self.result_filename = self.get_parameter('result_filename').value

        self.metrics_header = ['mission_name', 'datetime', 'metric']

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def save_metrics(self, data):
        result_path = Path(self.result_path).expanduser()

        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)

        result_file = result_path / (self.result_filename + '.csv')
        if result_file.is_file() is False:
            result_file.touch()
            self.append_csv(result_file, self.metrics_header)

        self.append_csv(result_file, data)

    def append_csv(self, file_path, data):
        with open(file_path, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def perform_mission(self):
        self.get_logger().warning("No mission defined!!!")
