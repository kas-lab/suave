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

"""Lifecycle node that commands the vehicle along an expanding spiral."""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from std_msgs.msg import Bool
from suave.bluerov_gazebo import BlueROVGazebo
from suave_msgs.action import SpiralSearch


def spiral_points(i, old_x, old_y, resolution=0.1, spiral_width=1.0):
    """Return the next spiral offset (x, y) from the spiral center."""
    if i == 0:
        return .0, .0
    else:
        delta_angle = i*resolution - (i-1)*resolution
        old_radius = math.sqrt(old_x**2 + old_y**2)
        current_radius = old_radius + (spiral_width*delta_angle/(2*math.pi))
        x = current_radius*math.cos(i*resolution)
        y = current_radius*math.sin(i*resolution)
        return x, y


class SpiralSearcherLC(Node):
    """Lifecycle node that executes an expanding spiral search pattern."""

    def __init__(self, node_name, **kwargs):
        """Create the spiral searcher node."""
        self._enabled = False
        self.spiral_count: int = 0
        self.spiral_x: float = 0.0
        self.spiral_y: float = 0.0
        # Absolute Gazebo/map-frame center, set on first publish.
        self.spiral_center_x = None
        self.spiral_center_y = None
        self.timer_period = 1.0
        self._timer: Optional[Timer] = None
        self.count = 0
        self.z_delta = 0
        self.old_spiral_altitude = -1
        self._pipeline_detected = False
        self._action_server = None

        super().__init__(node_name, **kwargs)

        self.goal_setpoint = None

        spiral_altitude_descriptor = ParameterDescriptor(
            description='Sets the spiral altitude of the UUV.')
        self.declare_parameter(
            'spiral_altitude', 2.0, spiral_altitude_descriptor)
        self.declare_parameter('use_action_server', False)

        self.param_change_callback_handle = \
            self.add_on_set_parameters_callback(self.param_change_callback)

        self.trigger_configure()

    def param_change_callback(self, parameters):
        """Log parameter changes."""
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            self.get_logger().info(
                "parameter '{}' is now: {}".format(
                    parameter.name,
                    parameter.value))
        return result

    def _pipeline_detected_cb(self, msg):
        """Set the internal flag when the pipeline is detected."""
        if msg.data:
            self._pipeline_detected = True

    def publish(self):
        """Publish the next spiral setpoint when the node is enabled."""
        if self._enabled is True:
            self.spiral_altitude = self.get_parameter(
                    'spiral_altitude').get_parameter_value().double_value

            # Initialize spiral center from the current MAVROS local pose the
            # first time a local position is available (map frame == Gazebo
            # frame, so this is the robot's actual start position).
            if self.spiral_center_x is None or self.spiral_center_y is None:
                if not self.ardusub.local_pos_received:
                    return
                local_pos = self.ardusub.local_pos.pose.position
                self.spiral_center_x = local_pos.x
                self.spiral_center_y = local_pos.y

            # Narrow to plain floats so arithmetic below is unambiguous.
            center_x = self.spiral_center_x
            center_y = self.spiral_center_y

            if self.old_spiral_altitude != self.spiral_altitude:
                self.spiral_count += 1

            self.old_spiral_altitude = self.spiral_altitude
            fov = math.pi/3
            spiral_width = 2.0*self.spiral_altitude*math.tan(fov/2)

            altitude_bug = False
            if self.goal_setpoint is not None and self.count > 10:
                altitude_bug = self.ardusub.check_setpoint_reached_xy(
                    self.goal_setpoint, 0.4) \
                    and (not self.ardusub.check_setpoint_reached(
                        self.goal_setpoint, 0.4))

            if self.count > 10:
                if altitude_bug is True:
                    self.z_delta -= 0.25
                self.ardusub.altitude = self.spiral_altitude + self.z_delta
                self.ardusub.setpoint_position_local(
                    center_x + self.spiral_x,
                    center_y + self.spiral_y,
                    fixed_altitude=True)
                self.count = 0

            if self.goal_setpoint is None or \
               self.ardusub.check_setpoint_reached(self.goal_setpoint, 0.4):

                x, y = spiral_points(
                    self.spiral_count,
                    self.spiral_x,
                    self.spiral_y,
                    resolution=0.1,
                    spiral_width=spiral_width)

                self.ardusub.altitude = self.spiral_altitude + self.z_delta
                self.goal_setpoint = self.ardusub.setpoint_position_local(
                    center_x + x,
                    center_y + y,
                    fixed_altitude=True)
                if self.goal_setpoint is not None:
                    # Store the true target z (without the bug-correction
                    # offset) so that check_setpoint_reached detects whether
                    # the vehicle has reached the intended altitude, not the
                    # corrected one.
                    self.goal_setpoint.pose.position.z -= self.z_delta
                    self.spiral_count += 1
                    self.spiral_x = x
                    self.spiral_y = y
                self.count = 0
            else:
                self.count += 1

    def _goal_callback(self, goal_request):
        """Accept goals only when active and use_action_server is True."""
        if self._state_machine.current_state[1] != 'active':
            self.get_logger().warn('Goal rejected: node is not active.')
            return GoalResponse.REJECT
        if not self.get_parameter(
                'use_action_server').get_parameter_value().bool_value:
            self.get_logger().warn('Goal rejected: use_action_server is False.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Allow cancellation at any time."""
        return CancelResponse.ACCEPT

    def _execute_spiral_search(self, goal_handle):
        """Run spiral search until pipeline detected or cancelled."""
        self._pipeline_detected = False
        self._enabled = True
        start_time = time.time()
        rate = self.create_rate(1)

        while not self._pipeline_detected:
            if goal_handle.is_cancel_requested:
                self._enabled = False
                goal_handle.canceled()
                result = SpiralSearch.Result()
                result.time_spent = float(time.time() - start_time)
                result.pipeline_found = False
                return result

            feedback = SpiralSearch.Feedback()
            feedback.elapsed_time = float(time.time() - start_time)
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        self._enabled = False
        result = SpiralSearch.Result()
        result.time_spent = float(time.time() - start_time)
        result.pipeline_found = True
        goal_handle.succeed()
        return result

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node, start the spiral timer, and register the action server."""
        self.get_logger().info('on_configure() is called.')
        self.ardusub = BlueROVGazebo('bluerov_spiral_search')

        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self._pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self._pipeline_detected_cb, 10)

        self._action_server = ActionServer(
            self,
            SpiralSearch,
            'spiral_search',
            execute_callback=self._execute_spiral_search,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._timer_ = self.create_timer(self.timer_period, self.publish)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate: start spiral immediately unless action server mode is on."""
        self.get_logger().info("on_activate() is called.")
        self.spiral_center_x = None
        self.spiral_center_y = None
        use_action_server = self.get_parameter(
            'use_action_server').get_parameter_value().bool_value
        if not use_action_server:
            self._enabled = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node and stop spiral behavior."""
        self.get_logger().info("on_deactivate() is called.")
        self._enabled = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


def main():
    """Run the spiral searcher lifecycle node."""
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = SpiralSearcherLC('f_generate_search_path_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
