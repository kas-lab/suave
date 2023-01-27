#!/usr/bin/env python
import sys
import rclpy
import threading

from rclpy.node import Node
from rclpy.duration import Duration

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mavros_msgs.msg import State
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters


class ThrusterMonitor(Node):
    def __init__(self):
        super().__init__('thruster_monitor_node')

        # '(thrusterN, failure/recovery, delta time in seconds )'
        # e.g. '(1, failure, 50)'
        self.declare_parameter('thruster_events', [''])
        self.thruster_events = self.read_thruster_events(
            self.get_parameter('thruster_events').value)

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.mavros_state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

    def status_cb(self, msg):
        if msg.mode == 'GUIDED':
            self.last_event_time = self.get_clock().now().to_msg().sec
            self.thruster_event_timer = self.create_timer(
                1, self.thruster_event_cb)
            self.destroy_subscription(self.mavros_state_sub)

    def read_thruster_events(self, events):
        thruster_events = []
        if events != '':
            for event in events:
                if event != '':
                    thruster_events.append(
                        [e.strip() for e in event.strip('()').split(',')])
        return thruster_events

    def thruster_event_cb(self):
        current_time = self.get_clock().now().to_msg().sec
        delta_time = current_time - self.last_event_time
        if len(self.thruster_events) > 0 and \
           delta_time >= int(self.thruster_events[0][2]):

            self.change_thruster_status(
                self.thruster_events[0][0], self.thruster_events[0][1])
            self.last_event_time = self.get_clock().now().to_msg().sec
            self.thruster_events.pop(0)

    def change_thruster_status(self, thruster, value):
        parameter = Parameter()
        parameter.name = 'SERVO' + thruster + '_FUNCTION'
        parameter.value.type = ParameterType.PARAMETER_INTEGER

        print_status = ''
        diagnostic_value = ''
        if value == 'failure':
            parameter.value.integer_value = 0
            print_status = 'failed'
            diagnostic_value = 'FALSE'
        elif value == 'recovery':
            parameter.value.integer_value = int(thruster) + 32
            print_status = 'recovered'
            diagnostic_value = 'RECOVERED'
        else:
            self.get_logger().info(
                'Wrong event value: {}. '.format(value) +
                'Values supported are failure and recovery')
            return

        req = SetParameters.Request()
        req.parameters.append(parameter)
        self.get_logger().info('Thruster {0} {1}'.format(
            thruster, print_status))
        self.call_service(SetParameters, 'mavros/param/set_parameters', req)

        key_value = KeyValue()
        key_value.key = 'c_thruster_{}'.format(thruster)
        key_value.value = diagnostic_value

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ''
        status_msg.message = 'Component status'
        status_msg.values.append(key_value)

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)

        # TODO: wait service to complete before changing the component state
        self.diagnostics_publisher.publish(diag_msg)

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        future = service.call_async(request)
        return future


def main():
    print("Starting thruster monitor node")

    rclpy.init(args=sys.argv)

    thruster_monitor = ThrusterMonitor()
    rclpy.spin(thruster_monitor)

    rclpy.shutdown()
