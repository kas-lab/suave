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

"""Define the pipeline search-and-inspection mission."""

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool
from std_srvs.srv import Empty


from suave_missions.mission_planner import MissionPlanner

from diagnostic_msgs.msg import DiagnosticArray


class InspectionMission(MissionPlanner):
    """Coordinate vehicle startup, pipeline search, and inspection tasks."""

    def __init__(self, node_name='inspection_mission'):
        """Initialize vehicle state, mission events, and MAVROS clients."""
        super().__init__(node_name)

        self.status = State()
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.status_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.arm_motors_service = self.create_client(
            CommandBool, 'mavros/cmd/arming')
        self.set_mode_service = self.create_client(
            SetMode, 'mavros/set_mode')

        self.battery_sub = self.create_subscription(
            DiagnosticArray,
            'diagnostics',
            self.battery_level_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.declare_parameter('battery_constraint', False)
        self.declare_parameter('battery_constraint_value', 0.2)

    def perform_mission(self):
        """Arm the vehicle, enter guided mode, and execute mission tasks."""
        self.get_logger().info("Pipeline inspection mission starting!!")
        self.timer = self.create_rate(1)

        while not self.status.armed:
            self.get_logger().info(
                'BlueROV is armed: {}'.format(self.status.armed))
            self.arm_motors(True)
            self.timer.sleep()

        guided_mode = 'GUIDED'
        while self.status.mode != guided_mode:
            self.get_logger().info(
                'BlueROV mode is: {} waiting for GUIDED mode'.format(
                 self.status.mode))
            self.set_mode(guided_mode)
            self.timer.sleep()

        self.mission_start_time = self.get_clock().now()

        self.perform_task('search_pipeline', lambda: self.pipeline_detected)
        self.perform_task('inspect_pipeline', lambda: self.pipeline_inspected)

    def battery_level_cb(self, msg):
        """Abort and save when the battery constraint is violated."""
        battery_constraint_arg = self.get_parameter('battery_constraint').value
        if battery_constraint_arg is True:
            for status in msg.status:
                if status.message == 'QA status':
                    for value in status.values:
                        if value.key == 'battery_level':
                            constraint = self.get_parameter(
                                'battery_constraint_value').value
                            if float(value.value) < constraint:
                                self.get_logger().warn(
                                    "Low battery! Mission abort.")
                                self.abort_mission = True
                                self.call_service(
                                    self.save_mission_results_cli,
                                    Empty.Request()
                                )
                                self.destroy_subscription(self.battery_sub)
                            break

    def status_cb(self, msg):
        """Update the latest MAVROS vehicle state."""
        self.status = msg

    def pipeline_detected_cb(self, msg):
        """Update pipeline detection state and record first detection time."""
        self.pipeline_detected = msg.data
        if self.pipeline_detected is True:
            self.pipeline_detected_time = self.get_clock().now()
            self.destroy_subscription(self.pipeline_detected_sub)

    def pipeline_inspected_cb(self, msg):
        """Update whether pipeline inspection has completed."""
        self.pipeline_inspected = msg.data

    def arm_motors(self, arm_motors_bool):
        """Request the desired motor arming state."""
        req = CommandBool.Request()
        req.value = arm_motors_bool
        return self.call_service(self.arm_motors_service, req)

    def set_mode(self, mode):
        """Request a MAVROS custom mode."""
        req = SetMode.Request()
        req.custom_mode = mode
        return self.call_service(self.set_mode_service, req)
