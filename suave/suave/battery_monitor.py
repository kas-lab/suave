# Copyright 2023 Gustavo Rezende Silva
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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mavros_msgs.msg import State
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

import rclpy
from rclpy.node import Node
import sys


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('qa_publishing_period', 1.0)
        self.declare_parameter('discharge_time', 200.0)

        self.qa_publishing_period = self.get_parameter(
            'qa_publishing_period').value

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.battery_recharged_pub = self.create_publisher(
            Bool, '/battery_monitor/recharge/complete', 10)

        self.battery_level = 1.0

        self.get_interpolated_path_srv = self.create_service(
            Trigger,
            'battery_monitor/recharge',
            self.recharge_battery_cb)

        self.mavros_state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

    def status_cb(self, msg):
        if msg.mode == "GUIDED":
            self.last_time = self.get_clock().now().to_msg().sec
            self.qa_publisher_timer = self.create_timer(
                self.qa_publishing_period, self.qa_publisher_cb)
            self.destroy_subscription(self.mavros_state_sub)

    def qa_publisher_cb(self):
        discharge_time = self.get_parameter('discharge_time').value

        current_time = self.get_clock().now().to_msg().sec
        dt = current_time - self.last_time

        if dt > 0:
            self.last_time = current_time

        v = self.battery_level - (1/discharge_time)*dt
        if v < 0.0:
            v = 0.0
        self.battery_level = v

        key_value = KeyValue()
        key_value.key = "battery_level"
        key_value.value = str(self.battery_level)

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ""
        status_msg.message = "QA status"
        status_msg.values.append(key_value)

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)

        self.diagnostics_publisher.publish(diag_msg)

    def recharge_battery_cb(self, req, res):
        self.battery_level = 1.0
        res.success = True
        self.battery_recharged_pub.publish(Bool(data=True))
        return res


def main():
    print("Starting battery_monitor observer node")

    rclpy.init(args=sys.argv)

    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)

    rclpy.shutdown()
