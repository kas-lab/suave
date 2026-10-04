# Copyright 2026 KAS Lab
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

"""Publish a sinusoidal horizontal ocean current with a varying heading."""

import math
import random

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from geometry_msgs.msg import Point

from mavros_msgs.msg import State

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node


def gauss_markov_step(state, dt, time_constant, rng):
    """Advance a unit-variance Gauss-Markov process exactly by dt seconds."""
    decay = math.exp(-dt / time_constant)
    noise = math.sqrt(1.0 - decay * decay) * rng.gauss(0.0, 1.0)
    return decay * state + noise


class WaterCurrent(Node):
    """Publish a low-impact sinusoidal current vector."""

    def __init__(self):
        """Initialize water-current parameters, publisher, and timer."""
        super().__init__('water_current')
        self.publisher = self.create_publisher(Point, '/ocean_current', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.initial_time = None
        self.mavros_state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

        self.declare_parameter(
            'mean_current',
            0.05,
            ParameterDescriptor(
                description='Mean current speed in m/s.'),
        )
        self.declare_parameter(
            'amplitude',
            0.03,
            ParameterDescriptor(
                description='Sinusoidal current speed amplitude in m/s.'),
        )
        self.declare_parameter(
            'period',
            90.0,
            ParameterDescriptor(
                description=(
                    'Sinusoidal current oscillation period in seconds.')),
        )
        self.declare_parameter(
            'heading',
            0.0,
            ParameterDescriptor(
                description='Mean current heading in degrees or radians.'),
        )
        self.declare_parameter(
            'heading_unit',
            'degrees',
            ParameterDescriptor(
                description=(
                    'Unit for heading, heading_amplitude, and '
                    'heading_noise_std: degrees or radians.')),
        )
        self.declare_parameter(
            'heading_amplitude',
            10.0,
            ParameterDescriptor(
                description='Sinusoidal heading amplitude in heading_unit.'),
        )
        self.declare_parameter(
            'heading_period',
            120.0,
            ParameterDescriptor(
                description=(
                    'Sinusoidal heading oscillation period in seconds.')),
        )
        self.declare_parameter(
            'heading_noise_std',
            0.0,
            ParameterDescriptor(
                description=(
                    'Stationary standard deviation of the Gauss-Markov '
                    'heading noise in heading_unit; 0 disables it.')),
        )
        self.declare_parameter(
            'heading_noise_time_constant',
            30.0,
            ParameterDescriptor(
                description=(
                    'Correlation time constant of the Gauss-Markov heading '
                    'noise in seconds.')),
        )
        self.declare_parameter(
            'heading_noise_seed',
            0,
            ParameterDescriptor(
                description=(
                    'Random seed for the heading noise; read once at '
                    'startup.')),
        )
        self.declare_parameter(
            'publish_period',
            1.0,
            ParameterDescriptor(
                description='Current vector publishing period in seconds.'),
        )

        self.rng = random.Random(self.get_parameter(
            'heading_noise_seed').get_parameter_value().integer_value)
        # Unit-variance Gauss-Markov state, scaled by heading_noise_std.
        self.heading_noise_state = 0.0
        self.last_elapsed = 0.0

        publish_period = self.get_parameter(
            'publish_period').get_parameter_value().double_value
        self.timer = self.create_timer(publish_period, self.publish_current)
        self.get_logger().info('Publishing ocean current on /ocean_current')

    def status_cb(self, msg):
        """Start advancing the current schedule when GUIDED is reached."""
        if msg.mode == 'GUIDED' and self.initial_time is None:
            self.initial_time = self.get_clock().now()
            self.destroy_subscription(self.mavros_state_sub)

    def publish_current(self):
        """Publish the current vector projected onto horizontal axes."""
        mean_current = self.get_parameter(
            'mean_current').get_parameter_value().double_value
        amplitude = self.get_parameter(
            'amplitude').get_parameter_value().double_value
        period = self.get_parameter(
            'period').get_parameter_value().double_value

        if self.initial_time is None:
            elapsed = 0.0
        else:
            now = self.get_clock().now()
            elapsed = (now - self.initial_time).nanoseconds * 1e-9
        current_speed = mean_current
        if period > 0.0:
            current_speed += amplitude * math.sin(
                2.0 * math.pi * elapsed / period)
        else:
            self.get_logger().warn(
                'Ignoring sinusoidal component because period is not '
                'positive.',
                throttle_duration_sec=10.0,
            )

        heading = self.compute_heading(elapsed)

        current = Point()
        current.x = current_speed * math.cos(heading)
        current.y = current_speed * math.sin(heading)
        current.z = 0.0
        self.publisher.publish(current)
        self.publish_diagnostics(current)

    def compute_heading(self, elapsed):
        """Return the mean heading plus sinusoid and noise in radians."""
        heading = self.get_parameter(
            'heading').get_parameter_value().double_value
        heading_unit = self.get_parameter(
            'heading_unit').get_parameter_value().string_value.lower()
        heading_amplitude = self.get_parameter(
            'heading_amplitude').get_parameter_value().double_value
        heading_period = self.get_parameter(
            'heading_period').get_parameter_value().double_value
        heading_noise_std = self.get_parameter(
            'heading_noise_std').get_parameter_value().double_value
        heading_noise_time_constant = self.get_parameter(
            'heading_noise_time_constant').get_parameter_value().double_value

        dt = elapsed - self.last_elapsed
        self.last_elapsed = elapsed
        if heading_noise_time_constant > 0.0:
            if dt > 0.0:
                self.heading_noise_state = gauss_markov_step(
                    self.heading_noise_state,
                    dt,
                    heading_noise_time_constant,
                    self.rng,
                )
        elif heading_noise_std > 0.0:
            self.get_logger().warn(
                'Ignoring heading noise because heading_noise_time_constant '
                'is not positive.',
                throttle_duration_sec=10.0,
            )

        if heading_period > 0.0:
            heading += heading_amplitude * math.sin(
                2.0 * math.pi * elapsed / heading_period)
        elif heading_amplitude != 0.0:
            self.get_logger().warn(
                'Ignoring sinusoidal heading component because '
                'heading_period is not positive.',
                throttle_duration_sec=10.0,
            )
        if heading_noise_time_constant > 0.0:
            heading += heading_noise_std * self.heading_noise_state

        if heading_unit in ('degree', 'degrees', 'deg'):
            return math.radians(heading)
        if heading_unit not in ('radian', 'radians', 'rad'):
            self.get_logger().warn(
                'Unknown heading_unit; interpreting heading as radians.',
                throttle_duration_sec=10.0,
            )
        return heading

    def publish_diagnostics(self, current):
        """Publish the current vector as a diagnostic QA status."""
        key_value = KeyValue()
        key_value.key = 'water_current'
        key_value.value = f'[{current.x} {current.y} {current.z}]'

        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = 'water_current: Ocean current measurement'
        status_msg.message = 'QA status'
        status_msg.values.append(key_value)

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        diag_msg.status.append(status_msg)

        self.diagnostics_publisher.publish(diag_msg)


def main(args=None):
    """Run the water current publisher node."""
    rclpy.init(args=args)
    node = WaterCurrent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
