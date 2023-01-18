#!/usr/bin/env python
import sys
import rclpy
import threading

from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from pipeline_inspection_metacontrol.mission_planner import MissionPlanner


class MissionConstDist(MissionPlanner):
    def __init__(self, node_name='const_dist_mission_node'):
        super().__init__(node_name)
        self.mission_name = 'const_dist_mission'
        self.metrics_header = [
            'mission_name', 'datetime', 'initial pos (x,y)',
            'time_search (s)', 'time_mission (s)']

    def perform_mission(self):
        self.get_logger().info("Pipeline inspection mission starting!!")
        timer = self.create_rate(1)

        while not self.status.armed:
            self.get_logger().info(
                'BlueROV is armed: {}'.format(self.status.armed))
            self.arm_motors(True)
            timer.sleep()

        guided_mode = 'GUIDED'
        while self.status.mode != guided_mode:
            self.get_logger().info(
                'BlueROV mode is : {}'.format(self.status.mode))
            self.set_mode(guided_mode)
            timer.sleep()

        self.get_logger().info('Starting Search Pipeline task')

        mission_start_time = self.get_clock().now()
        generate_search_path_goal_future = self.send_adaptation_goal(
            'generate_search_path')

        while not self.pipeline_detected:
            timer.sleep()

        pipeline_detected_time = self.get_clock().now()

        generate_search_path_goal_handle = \
            generate_search_path_goal_future.result()
        generate_search_path_goal_handle.cancel_goal_async()
        self.get_logger().info('Task Search Pipeline completed')

        self.get_logger().info('Starting Inspect Pipeline task')
        inspect_pipeline_goal_future = self.send_adaptation_goal(
            'inspect_pipeline')

        while not self.pipeline_inspected:
            timer.sleep()

        mission_completed_time = self.get_clock().now()

        inspect_pipeline_goal_handle = inspect_pipeline_goal_future.result()
        inspect_pipeline_goal_handle.cancel_goal_async()
        self.get_logger().info('Task Inspect Pipeline completed')

        detection_time_delta = pipeline_detected_time - mission_start_time
        mission_time_delta = mission_completed_time - mission_start_time

        self.get_logger().info(
            'Time elapsed to detect pipeline {}'.format(
                detection_time_delta.to_msg().sec))
        self.get_logger().info(
            'Time elapsed to complete mission {}'.format(
                mission_time_delta.to_msg().sec))

        mission_data = [
            self.mission_name,
            datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
            '({0}, {1})'.format(
                round(self.initial_x, 2), round(self.initial_y, 2)),
            detection_time_delta.to_msg().sec,
            mission_time_delta.to_msg().sec]

        self.save_metrics(mission_data)


def main():

    rclpy.init(args=sys.argv)

    mission_node = MissionConstDist()

    mt_executor = MultiThreadedExecutor()
    thread = threading.Thread(
        target=rclpy.spin, args=[mission_node, mt_executor], daemon=True)
    thread.start()

    mission_node.perform_mission()

    thread.join()
    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
