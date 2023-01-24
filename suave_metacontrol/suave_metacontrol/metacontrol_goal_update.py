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
from rclpy.executors import MultiThreadedExecutor
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue



class MetacontrolGoalUpdate(Node):

    def __init__(self):
        super().__init__('metacontrol_goal_node')

        self.last_goal_sent = None
        subscriber_cb_group = MutuallyExclusiveCallbackGroup()
        client_cb_group = MutuallyExclusiveCallbackGroup()

        self.detect_subscription = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.detected_cb,
            10, callback_group= subscriber_cb_group)
        self.mros_action_client = ActionClient(self, ControlQos, 'mros_objective', callback_group=client_cb_group)

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool, 'pipeline/detected', self.pipeline_detected_cb, 10, callback_group=subscriber_cb_group)

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool, 'pipeline/inspected', self.pipeline_inspected_cb, 10, callback_group=subscriber_cb_group)

        self.time_monitor_timer = self.create_timer(2, self.send_adaptation_goal, callback_group=client_cb_group)
    
    
    
    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    
    def send_adaptation_goal(self, nfrs=[]):
        self.mros_action_client.wait_for_server()

        if (not self.pipeline_detected and self.last_goal_sent != "generate_search_path"):
            adaptation_goal = "generate_search_path"
        elif (self.pipeline_detected and not self.pipeline_inspected and self.last_goal_sent !="inspect_pipeline"):
            adaptation_goal = "inspect_pipeline"
        else:
            return
        #still need to change the action client to not be async I believe
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


def main(args=None):
    rclpy.init(args=args)

    metacontrol_goal_node = MetacontrolGoalUpdate()()

    rclpy.spin(metacontrol_goal_node)

    metacontrol_goal_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)
