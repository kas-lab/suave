#!/usr/bin/env python
import sys
import rclpy
import threading

from datetime import datetime
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from suave.mission_planner import MissionPlanner


class MissionConstDist(MissionPlanner):
    def __init__(self, node_name='const_dist_mission_node'):
        super().__init__(node_name)
        self.mission_name = 'metacontrol - const distance'
        self.metrics_header = [
            'mission_name', 'datetime', 'initial pos (x,y)',
            'time_search (s)', 'time_mission (s)']

        self.action_cb_group = ReentrantCallbackGroup()
        self.mros_action_client = ActionClient(
           self,
           ControlQos,
           'mros_objective',
           callback_group=self.action_cb_group)

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

        motion_goal_future = self.send_adaptation_goal('control_motion')
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
        motion_goal_handle = motion_goal_future.result()
        motion_goal_handle.cancel_goal_async()
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

    def send_adaptation_goal(self, adaptation_goal, nfrs=[]):
        self.mros_action_client.wait_for_server()

        goal_msg = ControlQos.Goal()

        goal_msg.qos_expected.objective_type = "f_" + str(adaptation_goal)
        goal_msg.qos_expected.objective_id = "obj_" + str(adaptation_goal) \
            + "_{:.0f}".format(self.get_clock().now().to_msg().sec / 10)
        goal_msg.qos_expected.selected_mode = ""
        for required_nfr in nfrs:
            nfr = KeyValue()
            nfr.key = str(required_nfr[0])
            nfr.value = str(required_nfr[1])
            goal_msg.qos_expected.qos.append(nfr)

        self.get_logger().info(
            'Sending adaptation goal  {0}'.format(
                goal_msg.qos_expected.objective_type))
        action_future = self.mros_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Adaptation goal Sent!!!')

        return action_future

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(">> Feedback received:")
        self.get_logger().info(
            '    Solving: {0} of type {1}'.format(
                feedback.qos_status.objective_id,
                feedback.qos_status.objective_type))
        self.get_logger().info(
            '    Objective status: {0}'.format(
                feedback.qos_status.objective_status))
        self.get_logger().info('    QAs Status: ')
        for qos in feedback.qos_status.qos:
            self.get_logger().info(
                '      Key: {0} - Value {1}'.format(qos.key, qos.value))
        self.get_logger().info(
            '    Current Function Grounding: {0}'.format(
                feedback.qos_status.selected_mode))


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
