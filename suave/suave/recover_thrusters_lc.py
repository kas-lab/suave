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

"""Lifecycle node that recovers disabled thrusters."""

import rclpy
import sys

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters

from suave_msgs.action import RecoverThrusters


class RecoverThrustersLC(Node):
    """Lifecycle node that re-enables all six thrusters via MAVROS."""

    def __init__(self, node_name, **kwargs):
        """Create the recover thrusters node."""
        super().__init__(node_name, **kwargs)
        self.set_parameters_service = None
        self._action_server = None
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node: create publisher, parameter client, and action server."""
        self.get_logger().info('on_configure() is called.')
        self.declare_parameter('use_action_server', False)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.set_parameters_service = self.create_client(
            SetParameters, 'mavros/param/set_parameters')
        self._action_server = ActionServer(
            self,
            RecoverThrusters,
            'recover_thrusters',
            execute_callback=self._execute_recover,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        return TransitionCallbackReturn.SUCCESS

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

    def _execute_recover(self, goal_handle):
        """Run thruster recovery and return result."""
        recovered = self.recover_thrusters()
        result = RecoverThrusters.Result()
        result.success = recovered
        if recovered:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate: start recovery immediately unless action server mode is on."""
        self.get_logger().info("on_activate() is called.")
        use_action_server = self.get_parameter(
            'use_action_server').get_parameter_value().bool_value
        if not use_action_server:
            self.executor.create_task(self.recover_thrusters)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.get_logger().info('on_cleanup() is called.')
        self.destroy_set_parameters_service()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self.destroy_set_parameters_service()
        return TransitionCallbackReturn.SUCCESS

    def destroy_set_parameters_service(self):
        """Destroy the MAVROS parameter client."""
        if self.set_parameters_service is not None:
            self.destroy_client(self.set_parameters_service)
            self.set_parameters_service = None

    def call_service(self, cli, request):
        """Call a ROS service synchronously, returning None on failure."""
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()

    def recover_thrusters(self):
        """Set all six thruster SERVO_FUNCTION parameters via MAVROS and publish diagnostics."""
        publish_rate = self.create_rate(4)
        rate = self.create_rate(0.1)
        rate.sleep()
        all_recovered = True
        for thruster in range(1, 7):
            parameter = Parameter()
            parameter.name = 'SERVO' + str(thruster) + '_FUNCTION'
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = thruster + 32

            req = SetParameters.Request()
            req.parameters.append(parameter)

            response = self.call_service(self.set_parameters_service, req)
            if response is None:
                all_recovered = False
                self.get_logger().error(
                    'Failed to recover thruster {}: no MAVROS parameter '
                    'response'.format(thruster))
                continue
            if not response.results:
                all_recovered = False
                self.get_logger().error(
                    'Failed to recover thruster {}: empty MAVROS parameter '
                    'response'.format(thruster))
                continue
            if not all(result.successful for result in response.results):
                all_recovered = False
                self.get_logger().error(
                    'Failed to recover thruster {}: MAVROS rejected '
                    'parameter update'.format(thruster))
                continue

            key_value = KeyValue()
            key_value.key = 'c_thruster_{}'.format(thruster)
            key_value.value = 'RECOVERED'

            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.name = ''
            status_msg.message = 'Component status'
            status_msg.values.append(key_value)

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            diag_msg.status.append(status_msg)

            self.diagnostics_publisher.publish(diag_msg)
            publish_rate.sleep()
        if all_recovered:
            self.get_logger().info("Thrusters recovered!")
        return all_recovered


def main():
    """Run the recover thrusters lifecycle node."""
    rclpy.init(args=sys.argv)
    recover_thrusters_node = RecoverThrustersLC('f_maintain_motion_node')
    mt_executor = MultiThreadedExecutor()
    rclpy.spin(recover_thrusters_node, mt_executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
