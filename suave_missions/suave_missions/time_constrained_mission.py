#!/usr/bin/env python
import sys
import rclpy
import os

from datetime import datetime
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from suave_missions.inspection_mission import InspectionMission


class MissionTimeConstrained(InspectionMission):
    def __init__(self, node_name='time_contrained_mission'):
        super().__init__(node_name)
        if self.result_filename == 'mission_results':
            self.result_filename = 'time_mission_results'

        self.mission_name = 'time constrained ' + str(self.adaptation_manager)
        self.metrics_header = [
            'mission_name',
            'datetime',
            'initial pos (x,y)',
            'time budget (s)',
            'time search (s)',
            'distance inspected (m)']

        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value

        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.distance_inspected = 0.0
        self.time_monitor_timer = self.create_timer(0.5, self.time_monitor_cb)

    def distance_inspected_cb(self, msg):
        if self.abort_mission is False:
            self.distance_inspected = msg.data

    def time_monitor_cb(self):
        if self.mission_start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.mission_start_time
            if elapsed_time.to_msg().sec >= self.time_limit:
                self.abort_mission = True
                self.time_monitor_timer.destroy()

    def exit_mission(self):
        detection_time_delta = -1
        if self.pipeline_detected_time is not None:
            detection_time_delta = \
                self.pipeline_detected_time - self.mission_start_time
            detection_time_delta = detection_time_delta.to_msg().sec

        self.get_logger().info(
            'Time elapsed to detect pipeline: {} seconds'.format(
                detection_time_delta))
        self.get_logger().info(
            'Distance inspected: {} meters'.format(
                self.distance_inspected))

        mission_metrics = [
            self.mission_name,
            datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
            '({0}, {1})'.format(
                round(self.initial_x, 2), round(self.initial_y, 2)),
            self.time_limit,
            detection_time_delta,
            self.distance_inspected]

        self.save_metrics(mission_metrics)
        os.system("touch ~/suave_ws/mission.done")


def main():
    rclpy.init(args=sys.argv)

    mission_node = MissionTimeConstrained()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(mission_node)
    mt_executor.create_task(mission_node.perform_mission)
    mt_executor.spin()

    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
