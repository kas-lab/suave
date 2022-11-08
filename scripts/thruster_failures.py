#!/usr/bin/env python
import sys
import rclpy
import threading

from rclpy.node import Node
from rclpy.duration import Duration

from mavros_wrapper.ardusub_wrapper import *

if __name__ == '__main__':
    print("Starting thrusters failure node")

    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("thruster_failure_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    service_timer = ardusub.create_rate(10)
    while ardusub.status.armed is False:
        service_timer.sleep()

    rate = ardusub.create_rate(2)
    time_init = ardusub.get_clock().now()

    # TODO: this should be set with ROS params
    # Array [delta time(s), motor_number, value]
    thruster_event = [[5, 1, 0], [20, 1, 33]]
    index = 0
    try:
        while rclpy.ok() and index < len(thruster_event):
            time_now = ardusub.get_clock().now()
            duration_seconds = thruster_event[index][0]
            if (time_now - time_init) > Duration(seconds=duration_seconds):
                servo = 'SERVO' + str(thruster_event[index][1]) + '_FUNCTION'
                ardusub.set_mavros_param(
                    servo,
                    ParameterType.PARAMETER_INTEGER,
                    thruster_event[index][2])
                time_init = ardusub.get_clock().now()
                index += 1
            rate.sleep()
    except KeyboardInterrupt:
        pass

    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()
