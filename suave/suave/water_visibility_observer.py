#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mavros_msgs.msg import State

import math
import rclpy
from rclpy.node import Node
import sys


class WaterVisibilityObserver(Node):

    def __init__(self):
        super().__init__('water_visibility')

        self.declare_parameter('qa_publishing_period', 0.2)
        self.declare_parameter('water_visibility_period', 100)
        self.declare_parameter('water_visibility_min', 1.25)
        self.declare_parameter('water_visibility_max', 3.75)
        self.declare_parameter('water_visibility_sec_shift', 0.0)

        self.qa_publishing_period = self.get_parameter(
            'qa_publishing_period').value

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.mavros_state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

    def status_cb(self, msg):
        if msg.mode == "GUIDED":
            self.initial_time = self.get_clock().now().to_msg().sec
            self.qa_publisher_timer = self.create_timer(
                self.qa_publishing_period, self.qa_publisher_cb)
            self.destroy_subscription(self.mavros_state_sub)

    def qa_publisher_cb(self):
        water_visibility_period = self.get_parameter(
            'water_visibility_period').value
        water_visibility_min = self.get_parameter(
            'water_visibility_min').value
        water_visibility_max = self.get_parameter(
            'water_visibility_max').value
        water_visibility_amp = abs(
            water_visibility_max - water_visibility_min)/2
        sec_shift = self.get_parameter(
            'water_visibility_sec_shift').value

        current_time = self.get_clock().now().to_msg().sec
        t = current_time - self.initial_time
        v_delta = water_visibility_amp + water_visibility_min
        water_visibility = water_visibility_amp * math.cos(
            (2*math.pi/water_visibility_period)*(t + sec_shift)) + v_delta

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
