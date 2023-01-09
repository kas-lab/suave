from typing import Optional

import rclpy

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from pipeline_inspection.bluerov_gazebo import BlueROVGazebo

import std_msgs.msg
import numpy as np
import threading


def spiral_points(i, resolution=0.1, spiral_width=1.0):
    step_idx = resolution * i
    x = np.cos(step_idx) * spiral_width * step_idx
    y = np.sin(step_idx) * spiral_width * step_idx
    return x, y


class SpiralSearcherLC(Node):

    def __init__(self, node_name, **kwargs):
        self._enabled = False
        self.spiral_count: int = 0
        self._timer: Optional[Timer] = None

        super().__init__(node_name, **kwargs)

        self.spiral_width: float = 0.
        self.goal_setpoint = None

        param_descriptor = ParameterDescriptor(
            description='Sets the spiral width of the UUV.')
        self.declare_parameter('spiral_width', 1.0, param_descriptor)

        spiral_altitude_descriptor = ParameterDescriptor(
            description='Sets the spiral altitude of the UUV.')
        self.declare_parameter(
            'spiral_altitude', 2.0, spiral_altitude_descriptor)

        self.param_change_callback_handle = \
            self.add_on_set_parameters_callback(self.param_change_callback)

        self.trigger_configure()

    def param_change_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            self.get_logger().info(
                "parameter '{}' is now: {}".format(
                    parameter.name,
                    parameter.value))
        return result

    def publish(self):
        if self._enabled is True:
            self.spiral_width = self.get_parameter(
                    'spiral_width').get_parameter_value().double_value

            self.spiral_altitude = self.get_parameter(
                    'spiral_altitude').get_parameter_value().double_value

            if self.goal_setpoint is None or \
               self.ardusub.check_setpoint_reached(self.goal_setpoint, 0.4):

                x, y = spiral_points(
                    self.spiral_count,
                    resolution=0.1,
                    spiral_width=self.spiral_width)

                self.get_logger().info(
                        'setpoint_postion_local value {0}, {1}'.format(x, y))

                self.ardusub.altitude = self.spiral_altitude
                self.goal_setpoint = self.ardusub.setpoint_position_local(
                    x, y, fixed_altitude=True)
                if self.goal_setpoint is not None:
                    self.spiral_count += 1

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')
        self.ardusub = BlueROVGazebo('bluerov_spiral_search')

        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self._timer_ = self.create_timer(1.0, self.publish)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self._enabled = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self._enabled = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


def main():
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
