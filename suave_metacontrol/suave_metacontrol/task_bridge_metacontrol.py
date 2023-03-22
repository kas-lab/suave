#!/usr/bin/env python
import sys
import rclpy

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from mros2_msgs.action import ControlQos

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from suave.task_bridge import TaskBridge


class TaskBridgeMetacontrol(TaskBridge):
    def __init__(self):
        super().__init__()

        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.mros_action_client = ActionClient(
            self, ControlQos, '/mros/objective',
            callback_group=client_cb_group)

        self.current_objectives_handle = dict()
        self.task_functions_dict = {
            'search_pipeline': ['f_maintain_motion', 'f_generate_search_path'],
            'inspect_pipeline': ['f_maintain_motion', 'f_follow_pipeline'],
        }

    def forward_task_request(self, function):
        self.get_logger().info("Waiting for future to complete")
        future = self.send_mros_objective(function)
        while self.executor.spin_until_future_complete(
                future, timeout_sec=1.0):
            self.get_logger().info("Waiting for future to complete")
        self.current_objectives_handle[function] = future.result()
        return self.current_objectives_handle[function].accepted

    def forward_task_cancel_request(self, function):
        self.get_logger().info('cancel {}'.format(function))
        if function in self.current_objectives_handle:
            goal_handle = self.current_objectives_handle[function]
            future = goal_handle.cancel_goal_async()
            self.get_logger().info('cancel requested {}'.format(function))
            while self.executor.spin_until_future_complete(
                    future, timeout_sec=1.0):
                self.get_logger().info("Waiting for future to complete")
            del function
            if future is not None and goal_handle.status == 3:
                return True
            else:
                return False
        else:
            self.get_logger().info(
                'Function {} is not active'.format(function))
            return True

    def send_mros_objective(self, function, nfrs=[]):
        goal_msg = ControlQos.Goal()
        goal_msg.qos_expected.objective_type = str(function)
        goal_msg.qos_expected.objective_id = 'obj_' + str(function) \
            + '_{:.0f}'.format(self.get_clock().now().to_msg().sec / 10)
        goal_msg.qos_expected.selected_mode = ''
        for required_nfr in nfrs:
            nfr = KeyValue()
            nfr.key = str(required_nfr[0])
            nfr.value = str(required_nfr[1])
            goal_msg.qos_expected.qos.append(nfr)
        while not self.mros_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('waiting for /mros/objective action ...')
        self.get_logger().info(
            'Sending adaptation goal  {0}'.format(
                goal_msg.qos_expected.objective_type))
        action_future = self.mros_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('MROS objective {} sent!!!'.format(
            function))
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
    print("Starting task bridge node")

    rclpy.init(args=sys.argv)

    task_bridge_node = TaskBridgeMetacontrol()

    executor = MultiThreadedExecutor()
    rclpy.spin(task_bridge_node, executor=executor)

    task_bridge_node.destroy_node()
    rclpy.shutdown()
