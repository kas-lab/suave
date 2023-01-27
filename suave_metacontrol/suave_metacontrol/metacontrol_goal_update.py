import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool
import sys
import threading

from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from suave_missions.mission_planner import MissionPlanner

from std_msgs.msg import Float32

from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue



class MetacontrolGoalUpdate(Node):

    def __init__(self):
        super().__init__('metacontrol_goal_node')
        self.get_logger().info('\n\n Beginning of Node init \n\n')


        self.last_goal_sent = None
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        client_cb_group = MutuallyExclusiveCallbackGroup()
        timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.detect_subscription = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10, callback_group= subscriber_cb_group)

        self.detect_subscription = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10, callback_group= subscriber_cb_group)

        self.mros_action_client = ActionClient(self, ControlQos, 'mros_objective', callback_group=client_cb_group)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self.pipeline_detected_cb, 10, callback_group=subscriber_cb_group)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool, 'pipeline/inspected', self.pipeline_inspected_cb, 10, callback_group=subscriber_cb_group)

        self.time_monitor_timer = self.create_timer(2, self.send_adaptation_goal, callback_group=timer_cb_group)
        self.get_logger().info('\n\n End of Node init \n\n')

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
    
    
    
    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    
    def send_adaptation_goal(self, nfrs=[]):

        if (not self.pipeline_detected and self.last_goal_sent != "generate_search_path"):
            self.get_logger().info('\n\n Not Pipeline Detected and Not same goal as last time')
            adaptation_goal = "generate_search_path"
        elif (self.pipeline_detected and not self.pipeline_inspected and self.last_goal_sent !="inspect_pipeline"):
            self.get_logger().info('\n\n Yes Pipeline Detected and Not same goal as last time')
            adaptation_goal = "inspect_pipeline"
        else:
            self.get_logger().info('\n\n No need \n\n')
            return
        #still need to change the action client to not be async I believe
        self.last_goal_sent = adaptation_goal
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
        self.mros_action_client.wait_for_server()
        self.get_logger().info(
            'Sending adaptation goal  {0}'.format(
                goal_msg.qos_expected.objective_type))
        action_future = self.mros_action_client.send_goal(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Adaptation goal Sent!!!')

        return action_future


def main(args=None):
    rclpy.init(args=args)

    metacontrol_goal_node = MetacontrolGoalUpdate()

    executor = MultiThreadedExecutor()
    executor.add_node(metacontrol_goal_node)

    executor.spin()




    metacontrol_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()