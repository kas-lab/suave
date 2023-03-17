#!/usr/bin/env python
import sys
import rclpy
import os

from datetime import datetime
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from suave_missions.mission_planner import MissionPlanner


class MissionTimeConstrained(MissionPlanner):
    def __init__(self, node_name='time_contrained_mission_node'):
        super().__init__(node_name)
        self.get_logger().info('New log')

        self.mission_name = 'time constrained ' + str(self.adaptation_manager)
        self.metrics_header = [
            'mission_name', 'datetime', 'initial pos (x,y)', 'time budget (s)',
            'time search (s)', 'distance inspected (m)']

        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value

        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.mission_start_time = None
        self.pipeline_detected_time = None
        self.distance_inspected = 0.0

    def distance_inspected_cb(self, msg):
        self.distance_inspected = msg.data

    def time_monitor_cb(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.mission_start_time
        if elapsed_time.to_msg().sec >= self.time_limit:
            self.abort_mission = True
            detection_time_delta = -1
            if self.pipeline_detected_time is not None:
                detection_time_delta = \
                    self.pipeline_detected_time - self.mission_start_time
                detection_time_delta = detection_time_delta.to_msg().sec

            # TODO: get this automatically
            self.initial_x = 1.0
            self.initial_y = 1.0
            mission_metrics = [
                self.mission_name,
                datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
                '({0}, {1})'.format(
                    round(self.initial_x, 2), round(self.initial_y, 2)),
                self.time_limit,
                detection_time_delta,
                self.distance_inspected]
            self.save_metrics(mission_metrics)
            self.get_logger().info('Time limited reached. Mission aborted!')
            os.system("touch ~/suave_ws/mission.done")
            self.time_monitor_timer.destroy()

    def perform_mission(self):
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
                'BlueROV mode is : {} waiting for GUIDED mode'.format(
                 self.status.mode))
            self.set_mode(guided_mode)
            self.timer.sleep()

        self.mission_start_time = self.get_clock().now()
        self.time_monitor_timer = self.create_timer(0.5, self.time_monitor_cb)

        self.perform_task('search_pipeline', lambda: self.pipeline_detected)
        self.perform_task('inspect_pipeline', lambda: self.pipeline_inspected)

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
