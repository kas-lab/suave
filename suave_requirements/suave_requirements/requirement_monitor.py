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

"""Publish runtime fulfillment of the SUAVE RELAX requirements."""

from diagnostic_msgs.msg import DiagnosticArray

from rcl_interfaces.msg import ParameterDescriptor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Float32

from suave_msgs.msg import ReactionTime

from suave_requirements.fulfillment import MovingAverage
from suave_requirements.fulfillment import reaction_time_fulfillment
from suave_requirements.fulfillment import search_footprint_fulfillment
from suave_requirements.fulfillment import thruster_availability_fulfillment

# Must match REACTION_TIME_QOS in suave_metrics/mission_metrics.py.
REACTION_TIME_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

THRUSTER_AVAILABILITY = 'thruster_availability'
SEARCH_FOOTPRINT = 'search_footprint'
REACTION_TIME = 'adaptation_reaction_time'
REACTION_TIME_TYPES = [
    ReactionTime.ADAPTATION_THRUSTER,
    ReactionTime.ADAPTATION_WATER_VISIBILITY,
    ReactionTime.ADAPTATION_BATTERY,
]


def fulfillment_topic(requirement):
    """Return the fulfillment topic name for a requirement."""
    return '/requirements/' + requirement + '/fulfillment'


def get_diagnostic_value(msg, key):
    """Return the last float value for ``key`` in a DiagnosticArray."""
    value = None
    for status in msg.status:
        for key_value in status.values:
            if key_value.key == key:
                try:
                    value = float(key_value.value)
                except ValueError:
                    continue
    return value


class RequirementMonitor(Node):
    """Compute requirement fulfillment from SUAVE observations and metrics."""

    def __init__(self, node_name='requirement_monitor', **kwargs):
        """Declare parameters and create subscriptions and publishers."""
        super().__init__(node_name, **kwargs)

        self.declare_parameter(
            'required_thrusters', 6, ParameterDescriptor(
                description='Number of thrusters required by the motion '
                            'controller.'))
        self.declare_parameter(
            'thruster_availability_foot', 1.0, ParameterDescriptor(
                description='FuzzyLeftShoulder foot (missing thrusters) for '
                            'the thruster availability requirement.'))
        self.declare_parameter(
            'thruster_availability_shoulder', 0.0, ParameterDescriptor(
                description='FuzzyLeftShoulder shoulder (missing thrusters) '
                            'for the thruster availability requirement.'))
        self.declare_parameter(
            'search_footprint_foot', 0.0, ParameterDescriptor(
                description='FuzzyRightShoulder foot (m^2) for the search '
                            'footprint requirement.'))
        self.declare_parameter(
            'search_footprint_shoulder', 12.0, ParameterDescriptor(
                description='FuzzyRightShoulder shoulder (m^2) for the '
                            'search footprint requirement.'))
        self.declare_parameter(
            'reaction_time_exponent', 0.2, ParameterDescriptor(
                description='FuzzyExponentialDecay exponent (1/s) for the '
                            'adaptation reaction time requirement.'))
        self.declare_parameter(
            'reaction_time_window_size', 5, ParameterDescriptor(
                description='Number of most recent reaction events averaged '
                            'into the combined reaction time fulfillment.'))
        self.declare_parameter(
            'publishing_period', 1.0, ParameterDescriptor(
                description='Period in seconds for republishing the latest '
                            'fulfillment values.'))

        self.required_thrusters = self.get_parameter(
            'required_thrusters').value
        self.thruster_foot = self.get_parameter(
            'thruster_availability_foot').value
        self.thruster_shoulder = self.get_parameter(
            'thruster_availability_shoulder').value
        self.footprint_foot = self.get_parameter(
            'search_footprint_foot').value
        self.footprint_shoulder = self.get_parameter(
            'search_footprint_shoulder').value
        self.reaction_time_exponent = self.get_parameter(
            'reaction_time_exponent').value

        self.reaction_time_average = MovingAverage(
            self.get_parameter('reaction_time_window_size').value)
        self.reaction_counts = {t: 0 for t in REACTION_TIME_TYPES}

        # Fulfillment is unknown, and not published, until the first
        # measurement or valid reaction time is received.
        self.fulfillment = {
            THRUSTER_AVAILABILITY: None,
            SEARCH_FOOTPRINT: None,
            REACTION_TIME: None,
        }
        for reaction_type in REACTION_TIME_TYPES:
            self.fulfillment[REACTION_TIME + '/' + reaction_type] = None

        self.fulfillment_publishers = {
            requirement: self.create_publisher(
                Float32, fulfillment_topic(requirement), 10)
            for requirement in self.fulfillment
        }

        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_cb, 10)
        self.reaction_time_sub = self.create_subscription(
            ReactionTime,
            'mission_metrics/reaction_time',
            self.reaction_time_cb,
            REACTION_TIME_QOS)

        self.publish_timer = self.create_timer(
            self.get_parameter('publishing_period').value,
            self.publish_all)

    def diagnostics_cb(self, msg):
        """Update thruster and footprint fulfillment from diagnostics."""
        operational = get_diagnostic_value(msg, 'operational_thrusters')
        if operational is not None:
            self.update(
                THRUSTER_AVAILABILITY,
                thruster_availability_fulfillment(
                    self.required_thrusters,
                    operational,
                    foot=self.thruster_foot,
                    shoulder=self.thruster_shoulder))

        coverage_area = get_diagnostic_value(msg, 'coverage_area')
        if coverage_area is not None:
            self.update(
                SEARCH_FOOTPRINT,
                search_footprint_fulfillment(
                    coverage_area,
                    foot=self.footprint_foot,
                    shoulder=self.footprint_shoulder))

    def reaction_time_cb(self, msg):
        """Update reaction time fulfillment for a newly recorded reaction."""
        reaction_type = msg.adaptation_type
        if reaction_type not in self.reaction_counts:
            self.get_logger().warning(
                'Ignoring reaction time of unknown adaptation type '
                '"{}"'.format(reaction_type))
            return
        if (msg.status != ReactionTime.STATUS_VALID or
                msg.count <= self.reaction_counts[reaction_type]):
            return
        self.reaction_counts[reaction_type] = msg.count
        fulfillment = reaction_time_fulfillment(
            msg.latest, exponent=self.reaction_time_exponent)
        self.reaction_time_average.add(fulfillment)
        self.update(REACTION_TIME + '/' + reaction_type, fulfillment)
        self.update(REACTION_TIME, self.reaction_time_average.value)

    def update(self, requirement, fulfillment):
        """Store and immediately publish a requirement's fulfillment."""
        self.fulfillment[requirement] = fulfillment
        self.publish(requirement)

    def publish(self, requirement):
        """Publish a requirement's fulfillment if it is known."""
        fulfillment = self.fulfillment[requirement]
        if fulfillment is None:
            return
        self.fulfillment_publishers[requirement].publish(
            Float32(data=float(fulfillment)))

    def publish_all(self):
        """Republish every known fulfillment value."""
        for requirement in self.fulfillment:
            self.publish(requirement)


def main(args=None):
    """Run the requirement monitor node."""
    rclpy.init(args=args)
    node = RequirementMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
