import csv
import rclpy
from pathlib import Path
from datetime import datetime
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pipeline_inspection.bluerov_gazebo import BlueROVGazebo
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

        self.action_cb_group = ReentrantCallbackGroup()
        self.mros_action_client = ActionClient(
            self,
            ControlQos,
            'mros_objective',
            callback_group=self.action_cb_group)

        self.declare_parameter('result_path', '~/pipeline_inspection/results')
        self.declare_parameter('result_filename', 'mission_results')

        self.result_path = self.get_parameter('result_path').value

        self.result_filename = self.get_parameter('result_filename').value

        self.metrics_header = ['mission_name', 'datetime', 'metric']

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def send_adaptation_goal(self, adaptation_goal, nfrs=[]):
        self.mros_action_client.wait_for_server()

        goal_msg = ControlQos.Goal()

        goal_msg.qos_expected.objective_type = "f_" + str(adaptation_goal)
        goal_msg.qos_expected.objective_id = "obj_" + str(adaptation_goal) \
            + "_{:.0f}".format(self.get_clock().now().to_msg().sec / 10)
        goal_msg.qos_expected.selected_mode = ""
        for required_nfr in nfrs:
            nfr = KeyValue()
            nfr.key = str(required_nfr[0])
            nfr.value = str(required_nfr[1])
            goal_msg.qos_expected.qos.append(nfr)

        self.get_logger().info(
            'Sending adaptation goal  {0}'.format(
                goal_msg.qos_expected.objective_type))
        action_future = self.mros_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Adaptation goal Sent!!!')

        return action_future

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(">> Feedback received:")
        self.get_logger().info(
            '    Solving: {0} of type {1}'.format(
                feedback.qos_status.objective_id,
                feedback.qos_status.objective_type))
        self.get_logger().info(
            '    Objective status: {0}'.format(
                feedback.qos_status.objective_status))
        self.get_logger().info('    QAs Status: ')
        for qos in feedback.qos_status.qos:
            self.get_logger().info(
                '      Key: {0} - Value {1}'.format(qos.key, qos.value))
        self.get_logger().info(
            '    Current Function Grounding: {0}'.format(
                feedback.qos_status.selected_mode))

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
