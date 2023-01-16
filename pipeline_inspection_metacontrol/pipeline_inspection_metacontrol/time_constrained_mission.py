#!/usr/bin/env python
import sys
import rclpy
import threading

from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from pipeline_inspection_metacontrol.mission_planner import MissionPlanner

from std_msgs.msg import Float32


class MissionTimeConstrained(MissionPlanner):
    def __init__(self, node_name='time_contrained_mission_node'):
        super().__init__(node_name)
        self.mission_name = 'time_constrained_mission'
        self.metrics_header = [
            'mission_name', 'datetime', 'time budget (s)',
            'time search (s)', 'distance inspected (m)']

        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value
        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1)

        self.abort_mission = False

        self.mission_start_time = None
        self.pipeline_detected_time = None
        self.distance_inspected = -1

    def distance_inspected_cb(self, msg):
        self.distance_inspected = msg.data

    def time_monitor_cb(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.mission_start_time
        if elapsed_time.to_msg().sec >= self.time_limit:
            self.abort_mission = True
            self.cancel_current_goal()
            detection_time_delta = -1
            if self.pipeline_detected_time is not None:
                detection_time_delta = \
                    self.pipeline_detected_time - self.mission_start_time
                detection_time_delta = detection_time_delta.to_msg().sec

            mission_metrics = [
                self.mission_name,
                datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
                self.time_limit,
                detection_time_delta,
                self.distance_inspected]
            self.save_metrics(mission_metrics)
            self.get_logger().info('Time limited reached. Mission aborted!')
            self.time_monitor_timer.destroy()

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

        self.mission_start_time = self.get_clock().now()
        self.time_monitor_timer = self.create_timer(0.5, self.time_monitor_cb)
        if self.abort_mission is False:
            self.current_goal_future = self.send_adaptation_goal(
                'generate_search_path')
        else:
            return

        while not self.pipeline_detected:
            if self.abort_mission is True:
                return
            timer.sleep()

        self.pipeline_detected_time = self.get_clock().now()

        self.cancel_current_goal()
        self.get_logger().info('Task Search Pipeline completed')

        self.get_logger().info('Starting Inspect Pipeline task')
        if self.abort_mission is False:
            self.current_goal_future = self.send_adaptation_goal(
                'inspect_pipeline')
        else:
            return

        while not self.pipeline_inspected:
            if self.abort_mission is True:
                return
            timer.sleep()

        self.cancel_current_goal()
        self.get_logger().info('Task Inspect Pipeline completed')

    def cancel_current_goal(self):
        if self.current_goal_future is not None:
            self.current_goal_handle = self.current_goal_future.result()
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_future = None
            self.current_goal_handle = None


def main():

    rclpy.init(args=sys.argv)

    mission_node = MissionTimeConstrained()

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
