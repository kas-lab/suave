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

"""Navigate to the recharge station and trigger battery recharging."""

import threading
import time

from typing import Callable
from typing import Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from suave.action_server_utils import accept_cancel
from suave.action_server_utils import make_goal_callback
from suave.action_server_utils import use_action_server
from suave.action_server_utils import wait_for_action_completion
from suave.mavros_position_controller import MavrosPositionController
from suave.ros_service_utils import call_service_with_timeout
from suave_msgs.action import RechargeBattery as RechargeBatteryAction


class RechargeBattery(Node):
    """Lifecycle node that navigates to and triggers the recharge station."""

    def __init__(self, node_name, **kwargs):
        """Create the recharge battery node."""
        super().__init__(node_name, **kwargs)
        self.cli_group = MutuallyExclusiveCallbackGroup()
        self._position_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_server = None
        self._controller = None
        self.recharge_battery_cli = None
        self._legacy_recharge_stop = threading.Event()
        self._legacy_recharge_task = None
        self._deactivate_event = threading.Event()
        self._goal_executing = threading.Event()
        self.declare_parameter('ground_depth_gz', -20.0)
        self.declare_parameter('altitude', 1.25)
        self.declare_parameter(
            'recharge_station_gz_pos', [-3.0, -2.0, -19.5])
        self.declare_parameter('recharge_retry_rate', 2.0)
        self.declare_parameter('use_action_server', False)
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Create the configured clients and action server."""
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.recharge_battery_cli = self.create_client(
            Trigger,
            'battery_monitor/recharge',
            callback_group=self.cli_group
        )

        ground_depth = self.get_parameter(
            'ground_depth_gz').get_parameter_value().double_value
        self._controller = MavrosPositionController(
            self,
            ground_depth,
            self._position_callback_group,
        )

        self._action_server = ActionServer(
            self,
            RechargeBatteryAction,
            'recharge_battery',
            execute_callback=self._execute_recharge,
            goal_callback=make_goal_callback(self),
            cancel_callback=accept_cancel,
        )

        self.get_logger().info(self.get_name() + ': on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def _make_result(self, success: bool) -> RechargeBatteryAction.Result:
        """Create a RechargeBattery action result."""
        result = RechargeBatteryAction.Result()
        result.success = success
        return result

    def _get_recharge_station_pose(self) -> Pose:
        """Return the configured recharge station pose."""
        station_parameter = self.get_parameter(
            'recharge_station_gz_pos').get_parameter_value()
        station_position = station_parameter.double_array_value
        return Pose(position=Point(
            x=station_position[0],
            y=station_position[1],
            z=station_position[2],
        ))

    def _try_recharge_once(self) -> Optional[bool]:
        """Advance recharge once, returning None while it is in progress."""
        if self._controller is None:
            return None

        station_pose = self._get_recharge_station_pose()
        altitude = self.get_parameter(
            'altitude').get_parameter_value().double_value

        setpoint = self._controller.publish_gazebo_setpoint(
            station_pose, altitude)

        if setpoint is None:
            return None
        if not self._controller.is_xy_setpoint_reached(setpoint, 0.5):
            return None

        service_result = call_service_with_timeout(
            self, self.recharge_battery_cli, Trigger.Request())
        return service_result is not None and service_result.success

    def _run_recharge(
            self, stop_requested: Callable[[], bool]) -> Optional[bool]:
        """Run recharge until completion or an external stop request."""
        retry_rate = self.get_parameter(
            'recharge_retry_rate').get_parameter_value().double_value
        rate = self.create_rate(retry_rate)
        while not stop_requested():
            recharged = self._try_recharge_once()
            if recharged is not None:
                return recharged
            rate.sleep()
        return None

    def _execute_recharge(
            self, goal_handle: ServerGoalHandle
            ) -> RechargeBatteryAction.Result:
        """Navigate to recharge station and trigger recharging."""
        self._goal_executing.set()
        try:
            recharged = self._run_recharge(
                lambda: goal_handle.is_cancel_requested
                or self._deactivate_event.is_set())
            result = self._make_result(recharged is True)
            if recharged is None:
                if self._deactivate_event.is_set():
                    goal_handle.abort()
                else:
                    goal_handle.canceled()
            elif recharged:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self._goal_executing.clear()

    def _run_legacy_recharge(self) -> None:
        """Run recharge for legacy lifecycle mode."""
        recharged = self._run_recharge(self._legacy_recharge_stop.is_set)
        if recharged is False:
            self.get_logger().error('Battery recharge failed.')

    def _start_legacy_recharge(self) -> None:
        """Start one legacy recharge task if no task is running."""
        if (self._legacy_recharge_task is not None and
                not self._legacy_recharge_task.done()):
            return
        self._legacy_recharge_stop.clear()
        if self.executor is None:
            return
        self._legacy_recharge_task = self.executor.create_task(
            self._run_legacy_recharge)

    def _stop_legacy_recharge(self) -> None:
        """Request that the legacy recharge task stop."""
        self._legacy_recharge_stop.set()

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""
        self.get_logger().info(self.get_name() + ': on_activate() is called.')
        self._deactivate_event.clear()
        if not use_action_server(self):
            self._start_legacy_recharge()
        self.get_logger().info(
            self.get_name() + ': on_activate() is completed.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("on_deactivate() is called.")
        self._deactivate_event.set()
        self._stop_legacy_recharge()
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self._deactivate_event.set()
        self._stop_legacy_recharge()
        self._wait_for_legacy_recharge()
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def _wait_for_legacy_recharge(self) -> None:
        """Wait for the legacy task to observe its stop event."""
        while (self._legacy_recharge_task is not None and
               not self._legacy_recharge_task.done()):
            time.sleep(0.01)

    def _destroy_configured_entities(self) -> None:
        """Destroy entities owned by the configured lifecycle state."""
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self.recharge_battery_cli is not None:
            self.destroy_client(self.recharge_battery_cli)
            self.recharge_battery_cli = None
        if self._controller is not None:
            self._controller.destroy()
            self._controller = None

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self._deactivate_event.set()
        self._stop_legacy_recharge()
        self._wait_for_legacy_recharge()
        wait_for_action_completion(self)
        self._destroy_configured_entities()
        return super().on_shutdown(state)


def main(args=None):
    """Run the recharge battery lifecycle node."""
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        lc_node = RechargeBattery('generate_recharge_path_node')
        executor.add_node(lc_node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            executor.shutdown()
            lc_node.destroy_node()
        finally:
            executor.shutdown()
            lc_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
