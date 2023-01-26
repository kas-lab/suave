#!/usr/bin/env python
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import threading


class FakeManagedSystem(Node):

    def __init__(self):
        super().__init__('mock')

        self._action_client = ActionClient(self, ControlQos, '/mros/objective')

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        timer_period = 4.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.water_visibility = 3.5
        self.water_visibility_delta = -0.1

    def set_objectives(self):
        self.get_logger().info('Waiting for server')
        self._action_client.wait_for_server()

        mock_goal_msg = ControlQos.Goal()

        mock_goal_msg.qos_expected.objective_type = "f_generate_search_path"
        mock_goal_msg.qos_expected.objective_id = "obj_search_{:.0f}".format(
            self.get_clock().now().to_msg().sec / 10)
        mock_goal_msg.qos_expected.selected_mode = ""
        nfr = KeyValue()
        nfr.key = "water_visibility"
        nfr.value = str(0.35)
        mock_goal_msg.qos_expected.qos.append(nfr)

        self.get_logger().info(
            'Sending goal  {0}'.format(
                mock_goal_msg.qos_expected.objective_type))
        future = self._action_client.send_goal_async(
            mock_goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

        timer = self.create_rate(0.1)
        timer.sleep()

        self.get_logger().info('%%%%CANCEL GOAL')
        handle = future.result()
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        fake_goal_msg = ControlQos.Goal()
        fake_goal_msg.qos_expected.objective_type = "f_fake"
        fake_goal_msg.qos_expected.objective_id = "obj_fake_{:.0f}".format(
            self.get_clock().now().to_msg().sec / 10)
        fake_goal_msg.qos_expected.selected_mode = ""
        nfr2 = KeyValue()
        nfr2.key = "mockiness"
        nfr2.value = str(0.8)
        fake_goal_msg.qos_expected.qos.append(nfr2)

        self.get_logger().info(
            'Sending goal  {0}'.format(
                fake_goal_msg.qos_expected.objective_type))
        self._action_client.send_goal_async(
            fake_goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(">> Feedback received:")
        self.get_logger().info(
            '  Solving: {0} of type {1}'.format(
                feedback.qos_status.objective_id,
                feedback.qos_status.objective_type))
        self.get_logger().info(
            '  Objective status: {0}'.format(
                feedback.qos_status.objective_status))
        self.get_logger().info('  QAs Status: ')
        for qos in feedback.qos_status.qos:
            self.get_logger().info(
                '    Key: {0} - Value {1}'.format(qos.key, qos.value))
        self.get_logger().info(
            '  Current Function Grounding: {0}'.format(
                feedback.qos_status.selected_mode))

    def timer_callback(self):
        if self.water_visibility <= 1.25 or self.water_visibility > 3.75:
            self.water_visibility_delta = -self.water_visibility_delta

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ""
        key_value = KeyValue()
        key_value.key = "water_visibility"
        key_value.value = str(self.water_visibility)
        status_msg.values.append(key_value)
        status_msg.message = "QA status"
        diag_msg.status.append(status_msg)

        self.diagnostics_publisher.publish(diag_msg)

        self.water_visibility += self.water_visibility_delta


def main():
    print("Starting fake managed system node")

    rclpy.init(args=sys.argv)

    managed_system = FakeManagedSystem()
    thread = threading.Thread(
        target=rclpy.spin, args=(managed_system, ), daemon=True)
    thread.start()
    managed_system.set_objectives()

    thread.join()
    managed_system.destroy_node()
    rclpy.shutdown()
