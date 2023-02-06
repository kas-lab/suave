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
            10, callback_group=subscriber_cb_group)

        self.detect_subscription = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10, callback_group=subscriber_cb_group)

        self.mros_action_client = ActionClient(
            self, ControlQos, '/mros/objective',
            callback_group=client_cb_group)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=subscriber_cb_group)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=subscriber_cb_group)

        self.time_monitor_timer = self.create_timer(
            2, self.goal_timer_cb, callback_group=timer_cb_group)
        self.get_logger().info('\n\n End of Node init \n\n')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(">> Feedback received:")
        # self.get_logger().info(
        #     '    Solving: {0} of type {1}'.format(
        #         feedback.qos_status.objective_id,
        #         feedback.qos_status.objective_type))
        # self.get_logger().info(
        #     '    Objective status: {0}'.format(
        #         feedback.qos_status.objective_status))
        # self.get_logger().info('    QAs Status: ')
        for qos in feedback.qos_status.qos:
            do = "something"
            # self.get_logger().info(
            #     '      Key: {0} - Value {1}'.format(qos.key, qos.value))
        # self.get_logger().info(
        #     '    Current Function Grounding: {0}'.format(
        #         feedback.qos_status.selected_mode))

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def goal_timer_cb(self):
        if (not self.pipeline_detected and self.last_goal_sent !=
                "generate_search_path"):
            self.get_logger().info(
                '\n\n Not Pipeline Detected and Not same goal as last time')
            self.motion_future = self.send_adaptation_goal('maintain_motion')
            self.get_logger().info('!!!Motion goal sent!!!')
            adaptation_goal = "generate_search_path"
        elif (self.pipeline_detected and not self.pipeline_inspected and
                self.last_goal_sent != "follow_pipeline"):
            self.get_logger().info(
                '\n\n Yes Pipeline Detected and Not same goal as last time')
            self.cancel_current_goal()
            self.get_logger().info('\n\n Cancelling the previous goal')
            adaptation_goal = "follow_pipeline"
        elif (self.pipeline_detected and self.pipeline_inspected):
            if (self.last_goal_sent == 'follow_pipeline'):
                self.cancel_motion_goal()
                self.cancel_current_goal()
                self.last_goal_sent = 'no goal'
            # It will just kind of hang here unless the pipeline becomes
            # undetected or uninspected.
        else:
            #self.get_logger().info('\n\n No need to send goal \n\n')
            return
        # still need to change the action client to not be async I believe
        self.last_goal_sent = adaptation_goal
        self.send_adaptation_goal(adaptation_goal)

    def send_adaptation_goal(self, adaptation_goal, nfrs=[]):
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
        self.current_goal_future = self.mros_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Adaptation goal Sent!!!')

    def cancel_current_goal(self):
        if self.current_goal_future is not None:
            self.current_goal_handle = self.current_goal_future.result()
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_future = None
            self.current_goal_handle = None

    def cancel_motion_goal(self):
        if self.motion_future is not None:
            self.motion_goal_handle = self.motion_future.result()
            self.motion_goal_handle.cancel_goal_async()
            self.motion_future = None
            self.motion_goal_handle = None


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
