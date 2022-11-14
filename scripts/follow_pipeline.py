#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import threading

from mavros_wrapper.ardusub_wrapper import *

from rcl_interfaces.msg import ParameterType
from pipeline_inspection.srv import GetPath
from geometry_msgs.msg import Pose


class BlueROVGazeboNode(BlueROVArduSubWrapper):
    def __init__(self, node_name='bluerov_gz_mavros'):
        super().__init__(node_name)
        self.gazebo_pos_sub = self.create_subscription(
            Pose, 'model/bluerov2/pose', self.gazebo_pos_cb, 10)
        self.first_gazebo_pos_msg = True

        # TODO: make this a ros param
        self.altitude = 2

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gazebo_pos_msg and self.local_pos_received:
            self.gz_local_delta_pos = [
                self.local_pos.pose.position.x - msg.position.x,
                self.local_pos.pose.position.y - msg.position.y,
                self.local_pos.pose.position.z - msg.position.z,
            ]
            self.first_gazebo_pos_msg = False

    def convert_gz_to_local_pos(self, gz_pose):
        local_pose = Pose()
        local_pose.position.x = gz_pose.position.x + self.gz_local_delta_pos[0]
        local_pose.position.y = gz_pose.position.y + self.gz_local_delta_pos[1]
        local_pose.position.z = \
            gz_pose.position.z + self.gz_local_delta_pos[2] + self.altitude

        return local_pose


def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    custom_mode = "GUIDED"
    while ardusub.status.mode != custom_mode:
        ardusub.set_mode(custom_mode)
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed is False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    pipe_path = ardusub.call_service(
        GetPath, 'pipeline_inspection/get_path', GetPath.Request())

    timer = ardusub.create_rate(5)  # Hz

    while not pipe_path.done():
        timer.sleep()

    for gz_pose in pipe_path.result().path.poses:
        local_pose = ardusub.convert_gz_to_local_pos(gz_pose)
        ardusub.setpoint_position_local(
            x=local_pose.position.x,
            y=local_pose.position.y,
            z=local_pose.position.z)
        while not ardusub.check_setpoint_reached(ardusub.pose_stamped(local_pose.position.x, local_pose.position.y, local_pose.position.z), delta=0.2):
            timer.sleep()

    print("Mission completed")


if __name__ == '__main__':
    print("Starting Bluerov agent node")

    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVGazeboNode("follow_pipeline_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)

    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()
