#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

import math
import rclpy
from rclpy.node import Node
import sys


class WaterVisibilityObserver(Node):

    def __init__(self):
        super().__init__('water_visibility_node')

        self.initial_time = self.get_clock().now().to_msg().sec

        self.declare_parameter('qa_publishing_period', 0.2)
        self.declare_parameter('water_visibility_period', 100)
        self.declare_parameter('water_visibility_min', 0.25)
        self.declare_parameter('water_visibility_max', 2.5)

        qa_publishing_period = self.get_parameter('qa_publishing_period').value

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.qa_publisher_timer = self.create_timer(
            qa_publishing_period, self.qa_publisher_cb)

    def qa_publisher_cb(self):
        water_visibility_period = self.get_parameter(
            'water_visibility_period').value
        water_visibility_min = self.get_parameter(
            'water_visibility_min').value
        water_visibility_max = self.get_parameter(
            'water_visibility_max').value
        water_visibility_amp = abs(
            water_visibility_max - water_visibility_min)/2

        current_time = self.get_clock().now().to_msg().sec
        t = current_time - self.initial_time
        v_delta = water_visibility_amp + water_visibility_min
        water_visibility = water_visibility_amp * math.cos(
            (2*math.pi/water_visibility_period)*t) + v_delta

        key_value = KeyValue()
        key_value.key = "water_visibility"
        key_value.value = str(water_visibility)

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ""
        status_msg.message = "QA status"
        status_msg.values.append(key_value)

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)

        self.diagnostics_publisher.publish(diag_msg)


def main():
    print("Starting water_visibility observer node")

    rclpy.init(args=sys.argv)

    water_visibility_observer = WaterVisibilityObserver()
    rclpy.spin(water_visibility_observer)

    rclpy.shutdown()
