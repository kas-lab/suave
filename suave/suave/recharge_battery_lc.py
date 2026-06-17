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

"""Lifecycle node that navigates to the recharge station and triggers recharging."""

import rclpy
import threading

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

from suave.bluerov_gazebo import BlueROVGazebo
from suave_msgs.action import RechargeBattery as RechargeBatteryAction


def check_lc_active(func):
    """Decorate func to run only when the node is active."""
    def inner(*args, **kwargs):
        if args[0].active is True:
            return func(*args, **kwargs)
    return inner


class RechargeBattery(Node):
    """Lifecycle node that navigates to and triggers the recharge station."""

    def __init__(self, node_name, **kwargs):
        """Create the recharge battery node."""
        super().__init__(node_name, **kwargs)
        self.recharge_timer_period = 5.0
        self.active = False
        self.cli_group = MutuallyExclusiveCallbackGroup()
        self._action_server = None
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure: declare parameters, create clients, and register action server."""
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.declare_parameter(
            'recharge_station_gz_pos', [-3.0, -2.0, -19.5])
        self.declare_parameter('use_action_server', False)

        self.recharge_battery_cli = self.create_client(
            Trigger,
            'battery_monitor/recharge',
            callback_group=self.cli_group
        )

        self.ardusub = BlueROVGazebo('bluerov_recharge')

        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self.recharge_cb_timer = self.create_timer(
            self.recharge_timer_period, self.recharge_cb)

        self._action_server = ActionServer(
            self,
            RechargeBatteryAction,
            'recharge_battery',
            execute_callback=self._execute_recharge,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(self.get_name() + ': on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request):
        """Accept goals only when active and use_action_server is True."""
        if not self.active:
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

    def _execute_recharge(self, goal_handle):
        """Navigate to recharge station and trigger recharging."""
        rate = self.create_rate(2)
        while True:
            if goal_handle.is_cancel_requested:
                result = RechargeBatteryAction.Result()
                result.success = False
                goal_handle.canceled()
                return result

            recharge_station_gz_pos = self.get_parameter(
                'recharge_station_gz_pos').get_parameter_value()
            recharge_station_pos = recharge_station_gz_pos.double_array_value
            station_pose = Pose(position=Point(
                x=recharge_station_pos[0],
                y=recharge_station_pos[1],
                z=recharge_station_pos[2],
            ))

            setpoint = self.ardusub.setpoint_position_gz(
                station_pose, fixed_altitude=True)

            if setpoint is None:
                rate.sleep()
                continue

            if self.ardusub.check_setpoint_reached_xy(setpoint, 0.5):
                svc_result = self.call_service(
                    self.recharge_battery_cli, Trigger.Request())
                result = RechargeBatteryAction.Result()
                result.success = (
                    svc_result is not None and svc_result.success)
                goal_handle.succeed()
                return result

            rate.sleep()

    @check_lc_active
    def recharge_cb(self):
        """Timer callback: navigate to and trigger recharge station (legacy mode)."""
        recharge_station_gz_pos = self.get_parameter(
                'recharge_station_gz_pos').get_parameter_value()
        recharge_station_pos = recharge_station_gz_pos.double_array_value
        station_pose = Pose(position=Point(
                x=recharge_station_pos[0],
                y=recharge_station_pos[1],
                z=recharge_station_pos[2],
            )
        )

        setpoint = self.ardusub.setpoint_position_gz(
            station_pose,
            fixed_altitude=True)

        if setpoint is None:
            return

        if self.ardusub.check_setpoint_reached_xy(setpoint, 0.5):
            self.call_service(self.recharge_battery_cli, Trigger.Request())
            return

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""
        self.get_logger().info(self.get_name() + ': on_activate() is called.')
        self.active = True
        self.get_logger().info(
            self.get_name() + ': on_activate() is completed.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("on_deactivate() is called.")
        self.active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Clean up the node."""
        self.active = False
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self.recharge_cb_timer)
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shut down the node."""
        self.get_logger().info('on_shutdown() is called.')
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self.recharge_cb_timer)
        return super().on_shutdown(state)

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


def main(args=None):
    """Run the recharge battery lifecycle node."""
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        lc_node = RechargeBattery('generate_recharge_path_node')
        executor.add_node(lc_node)
        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            executor.shutdown()
            lc_node.destroy_node()
        finally:
            executor.shutdown()
            lc_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
