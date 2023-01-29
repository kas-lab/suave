#!/usr/bin/env python
import sys
import rclpy
import threading
import os
from datetime import datetime
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from system_modes_msgs.srv import ChangeMode

from suave_missions.mission_planner import MissionPlanner


class MissionConstDist(MissionPlanner):
    def __init__(self, node_name='const_dist_mission_node'):
        super().__init__(node_name)
        self.mission_name = 'const distance'
        self.metrics_header = [
            'mission_name', 'datetime', 'initial pos (x,y)',
            'time_search (s)', 'time_mission (s)']

        self.generate_path_sm_cli = self.create_client(
                ChangeMode,
                '/f_generate_search_path/change_mode')

        self.inspect_pipeline_sm_cli = self.create_client(
                ChangeMode,
                '/f_inspect_pipeline/change_mode')

        self.declare_parameter('f_generate_search_path_mode', 'fd_spiral_low')
        self.declare_parameter('f_inspect_pipeline_mode', 'fd_inspect_pipeline')
        
        self.chosen_search_mode = self.get_parameter('f_generate_search_path_mode').value
        self.chosen_inspect_mode = self.get_parameter('f_inspect_pipeline_mode').value
        self.using_no_adaptation = self.adaptation_manager == 'none'

        self.get_logger().info('DISTANCE MISSION')
        
    def search_task(self):
        self.get_logger().info('Starting Search Pipeline task')

        self.mission_start_time = self.get_clock().now()
        if self.using_no_adaptation: 
            self.manual_sysmode_change(self.chosen_search_mode,self.generate_path_sm_cli)

        while not self.pipeline_detected:
            self.timer.sleep()

        self.pipeline_detected_time = self.get_clock().now()

        self.get_logger().info('Task Search Pipeline completed')

    def inspect_task(self):
        self.get_logger().info('Starting Inspect Pipeline task')
        if self.using_no_adaptation: 
            self.manual_sysmode_change(self.chosen_inspect_mode,self.inspect_pipeline_sm_cli)
        while not self.pipeline_inspected:
            self.timer.sleep()

        self.mission_completed_time = self.get_clock().now()

        self.get_logger().info('Task Inspect Pipeline completed')

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
                'BlueROV mode is : {}'.format(self.status.mode))
            self.set_mode(guided_mode)
            self.timer.sleep()

        self.search_task()
        if self.using_no_adaptation: 
            self.manual_sysmode_change('fd_unground',self.generate_path_sm_cli)

        self.inspect_task()

        if self.using_no_adaptation: 
            self.manual_sysmode_change('fd_unground',self.inspect_pipeline_sm_cli)



        detection_time_delta = self.pipeline_detected_time - self.mission_start_time
        mission_time_delta = self.mission_completed_time - self.mission_start_time

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

        os.system("touch ~/suave_ws/mission.done")

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
