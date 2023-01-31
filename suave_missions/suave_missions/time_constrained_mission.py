#!/usr/bin/env python
import sys
import rclpy
import threading
import os
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from suave_missions.mission_planner import MissionPlanner
from system_modes_msgs.srv import ChangeMode

from std_msgs.msg import Float32

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


class MissionTimeConstrained(MissionPlanner):
    def __init__(self, node_name='time_contrained_mission_node'):
        super().__init__(node_name)
        self.get_logger().info('New log')

        self.mission_name = 'metacontrol - time constrained'
        self.metrics_header = [
            'mission_name', 'datetime', 'initial pos (x,y)', 'time budget (s)',
            'time search (s)', 'distance inspected (m)']

 
        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value
        self.get_logger().info('\n\n\n!!!TIME LIMIT:' + str(self.time_limit))

        self.declare_parameter('f_generate_search_path_mode', 'fd_spiral_low')
        self.declare_parameter('f_follow_pipeline_mode', 'fd_follow_pipeline')

        self.chosen_search_mode = self.get_parameter('f_generate_search_path_mode').value
        self.chosen_inspect_mode = self.get_parameter('f_follow_pipeline_mode').value

        self.get_logger().info('\n\n\n!!!Adaptation Manager:' + str(self.adaptation_manager))
        
        
        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1)

        self.abort_mission = False

        self.using_no_adaptation = self.adaptation_manager == 'none'
        self.mission_start_time = None
        self.pipeline_detected_time = None
        self.distance_inspected = -1

        self.generate_path_sm_cli = self.create_client(
                ChangeMode,
                '/f_generate_search_path/change_mode')

        self.inspect_pipeline_sm_cli = self.create_client(
                ChangeMode,
                '/f_follow_pipeline/change_mode')

        self.get_logger().info('TIME MISSION')


    def distance_inspected_cb(self, msg):
        self.distance_inspected = msg.data

    def time_monitor_cb(self):
        current_time = self.get_clock().now()
        elapsed_time = current_time - self.mission_start_time
        if elapsed_time.to_msg().sec >= self.time_limit:
            self.abort_mission = True
            if self.using_no_adaptation: 
                self.manual_sysmode_change("fd_unground", [self.generate_path_sm_cli, self.inspect_pipeline_sm_cli])
            detection_time_delta = -1
            if self.pipeline_detected_time is not None:
                detection_time_delta = \
                    self.pipeline_detected_time - self.mission_start_time
                detection_time_delta = detection_time_delta.to_msg().sec

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
            self.time_monitor_timer.destroy()
    
    def search_task(self):
        if self.abort_mission is False:
            if self.using_no_adaptation: 
                self.get_logger().info('Sanity check, should be none as adaptation')

                self.manual_sysmode_change(self.chosen_search_mode,self.generate_path_sm_cli)
            while not self.pipeline_detected:
                if self.abort_mission is True:
                    return
                self.timer.sleep()
        else:
            return

        self.pipeline_detected_time = self.get_clock().now()
        
        self.get_logger().info('Task Search Pipeline completed')

    def inspect_task(self):
        self.get_logger().info('Starting Inspect Pipeline task')
        if self.abort_mission is False:
            if self.using_no_adaptation: 
                self.manual_sysmode_change(self.chosen_inspect_mode,self.inspect_pipeline_sm_cli)
            while not self.pipeline_inspected:
                if self.abort_mission is True:
                    return
                self.timer.sleep()
        else:
            return
            

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

        self.get_logger().info('Starting Search Pipeline task')

        self.mission_start_time = self.get_clock().now()
        self.time_monitor_timer = self.create_timer(0.5, self.time_monitor_cb)
        
        self.search_task()
        if self.using_no_adaptation: 
            self.manual_sysmode_change('fd_unground',self.generate_path_sm_cli)

        self.inspect_task()
        if self.using_no_adaptation: 
            self.manual_sysmode_change('fd_unground',self.inspect_pipeline_sm_cli)
        
        os.system("touch ~/suave_ws/mission.done")

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
