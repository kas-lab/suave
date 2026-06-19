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
import threading
import time
from typing import Callable
from typing import Optional

from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
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

from suave.action_server_utils import accept_cancel
from suave.action_server_utils import make_goal_callback
from suave.action_server_utils import use_action_server
from suave.action_server_utils import wait_for_action_completion
from suave.ros_service_utils import call_service_with_timeout
from suave_msgs.action import RecoverThrusters


StopRequested = Callable[[], bool]


class RecoverThrustersLC(Node):
    """Lifecycle node that re-enables all six thrusters via MAVROS."""

    def __init__(self, node_name, **kwargs):
        """Create the recover thrusters node."""
        super().__init__(node_name, **kwargs)
        self.set_parameters_service = None
        self._action_server = None
        self._abort_event = threading.Event()
        self._goal_executing = threading.Event()
        self._legacy_recovery_finished = threading.Event()
        self._legacy_recovery_finished.set()
        self.recover_task = None

        self.declare_parameter('system_mode_marker', '__DEFAULT__')
        self.declare_parameter('use_action_server', False)

        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Create the configured publisher, client, and action server."""
        self.get_logger().info('on_configure() is called.')

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.set_parameters_service = self.create_client(
            SetParameters, 'mavros/param/set_parameters')
        self._action_server = ActionServer(
            self,
            RecoverThrusters,
            'recover_thrusters',
            execute_callback=self._execute_recover,
            goal_callback=make_goal_callback(self),
            cancel_callback=accept_cancel,
        )
        return TransitionCallbackReturn.SUCCESS

    def _make_result(self, success):
        """Create a RecoverThrusters action result."""
        result = RecoverThrusters.Result()
        result.success = success
        return result

    def _execute_recover(self, goal_handle):
        """Run thruster recovery and return result."""
        self._goal_executing.set()
        try:
            recovered = self._recover_thrusters(
                stop_requested=lambda: (
                    self._abort_event.is_set()
                    or goal_handle.is_cancel_requested
                ))
            result = self._make_result(recovered is True)
            if recovered is True:
                goal_handle.succeed()
            elif recovered is None:
                if self._abort_event.is_set():
                    goal_handle.abort()
                else:
                    goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        finally:
            self._goal_executing.clear()

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start recovery unless action server mode is on."""
        self.get_logger().info("on_activate() is called.")
        self._abort_event.clear()
        if not use_action_server(self):
            self._legacy_recovery_finished.clear()
            self.recover_task = self.executor.create_task(
                self._run_legacy_recovery)
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("on_deactivate() is called.")
        self._abort_event.set()
        if not self._wait_for_legacy_recovery():
            return TransitionCallbackReturn.FAILURE
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.get_logger().info('on_cleanup() is called.')
        self._abort_event.set()
        if not self._wait_for_legacy_recovery():
            return TransitionCallbackReturn.FAILURE
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self._abort_event.set()
        if not self._wait_for_legacy_recovery():
            return TransitionCallbackReturn.FAILURE
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        return TransitionCallbackReturn.SUCCESS

    def _destroy_configured_entities(self) -> None:
        """Destroy entities owned by the configured lifecycle state."""
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self.set_parameters_service is not None:
            self.destroy_client(self.set_parameters_service)
            self.set_parameters_service = None
        if self.diagnostics_publisher is not None:
            self.destroy_publisher(self.diagnostics_publisher)
            self.diagnostics_publisher = None

    def _run_legacy_recovery(self) -> None:
        """Run legacy recovery and always signal task completion."""
        try:
            self._recover_thrusters(self._abort_event.is_set)
        except Exception as exception:
            self.get_logger().error(
                'Legacy thruster recovery failed: {}'.format(exception))
        finally:
            self._legacy_recovery_finished.set()

    def _wait_for_legacy_recovery(self, timeout_sec: float = 2.0) -> bool:
        """Wait for a legacy recovery task to stop cooperatively."""
        if self._legacy_recovery_finished.wait(timeout=timeout_sec):
            return True
        self.get_logger().error('Legacy thruster recovery did not stop.')
        return False

    def _wait_interruptibly(
            self, duration: float, stop_requested: StopRequested,
            poll_interval: float = 0.05) -> bool:
        """Wait for a duration or return false when stopping is requested."""
        deadline = time.monotonic() + duration
        while True:
            if stop_requested():
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            self._abort_event.wait(
                timeout=min(poll_interval, remaining))

    def _recover_thrusters(
            self,
            stop_requested: Optional[StopRequested] = None,
            ) -> Optional[bool]:
        """Return true only when every thruster write succeeds."""
        if stop_requested is None:
            stop_requested = self._abort_event.is_set
        if not self._wait_interruptibly(10.0, stop_requested):
            return None

        all_recovered = True
        for thruster in range(1, 7):
            if stop_requested():
                return None
            parameter = Parameter()
            parameter.name = 'SERVO' + str(thruster) + '_FUNCTION'
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = thruster + 32

            req = SetParameters.Request()
            req.parameters.append(parameter)

            response = call_service_with_timeout(
                self,
                self.set_parameters_service,
                req,
                stop_requested=stop_requested,
            )
            if stop_requested():
                return None
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
            if not self._wait_interruptibly(0.25, stop_requested):
                return None
        if all_recovered:
            self.get_logger().info("Thrusters recovered!")
        return all_recovered


def main():
    """Run the recover thrusters lifecycle node."""
    rclpy.init(args=sys.argv)
    try:
        executor = MultiThreadedExecutor()
        recover_thrusters_node = RecoverThrustersLC('f_maintain_motion_node')
        executor.add_node(recover_thrusters_node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            executor.shutdown()
            recover_thrusters_node.destroy_node()
        finally:
            executor.shutdown()
            recover_thrusters_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
