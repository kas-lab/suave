#!/usr/bin/env python
import sys
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from pipeline_inspection.bluerov_gazebo import BlueROVGazebo
from std_msgs.msg import Bool


class MissionNode(BlueROVGazebo):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self.pipeline_detected_cb, 10)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool, 'pipeline/inspected', self.pipeline_inspected_cb, 10)

        self.mros_action_client = ActionClient(
            self, ControlQos, 'mros_objective')

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

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
        action_handle = self.mros_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Adaptation goal Sent!!!')

        return action_handle

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
        generate_search_path_goal_handle = self.send_adaptation_goal(
            'generate_search_path', [('water_visibility', 0.15)])

        while not self.pipeline_detected:
            timer.sleep()

        generate_search_path_goal_handle.cancel_goal_async()
        self.get_logger().info('Task Search Pipeline completed')

        self.get_logger().info('Starting Inspect Pipeline task')
        inspect_pipeline_goal_handle = self.send_adaptation_goal(
            'inspect_pipeline')

        while not self.pipeline_inspected:
            timer.sleep()

        inspect_pipeline_goal_handle.cancel_goal_async()
        self.get_logger().info('Task Inspect Pipeline completed')


def main():

    rclpy.init(args=sys.argv)

    mission_node = MissionNode()

    thread = threading.Thread(
        target=rclpy.spin, args=(mission_node, ), daemon=True)
    thread.start()

    mission_node.perform_mission()

    mission_node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
