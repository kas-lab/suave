import csv
import rclpy
from pathlib import Path
from datetime import datetime
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool
from suave_msgs.srv import Task


class MissionPlanner(Node):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)

        self.status = State()
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.status_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_detected = False
        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_inspected = False
        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.arm_motors_service = self.create_client(
            CommandBool, 'mavros/cmd/arming')
        self.set_mode_service = self.create_client(
            SetMode, 'mavros/set_mode')

        self.task_request_service = self.create_client(
            Task, 'task/request')
        self.task_cancel_service = self.create_client(
            Task, 'task/cancel')

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')
        self.declare_parameter('adaptation_manager', 'none')
        self.declare_parameter('mission_type', 'time_constrained_mission')

        self.result_path = self.get_parameter('result_path').value
        self.result_filename = self.get_parameter('result_filename').value

        self.adaptation_manager = self.get_parameter(
            'adaptation_manager').value
        self.mission_metric = self.get_parameter('mission_type').value

        self.metrics_header = ['mission_name', 'datetime', 'metric']

        self.mission_start_time = None
        self.pipeline_detected_time = None
        self.abort_mission = False

    def status_cb(self, msg):
        self.status = msg

    def pipeline_detected_cb(self, msg):
        self.pipeline_detected = msg.data
        if self.pipeline_detected is True:
            self.pipeline_detected_time = self.get_clock().now()
        self.destroy_subscription(self.pipeline_detected_sub)

    def pipeline_inspected_cb(self, msg):
        self.pipeline_inspected = msg.data

    def arm_motors(self, arm_motors_bool):
        req = CommandBool.Request()
        req.value = arm_motors_bool
        return self.call_service(self.arm_motors_service, req)

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        return self.call_service(self.set_mode_service, req)

    def request_task(self, task_name):
        req = Task.Request()
        req.task_name = task_name
        return self.call_service(self.task_request_service, req)

    def cancel_task(self, task_name):
        req = Task.Request()
        req.task_name = task_name
        return self.call_service(self.task_cancel_service, req)

    def call_service(self, cli, request):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = cli.call_async(request)
        while self.executor.spin_until_future_complete(
                future, timeout_sec=1.0):
            self.get_logger().info("Waiting for future to complete")
        return future.result()

    def save_metrics(self, data):
        result_path = Path(self.result_path).expanduser()

        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)

        result_file = result_path / (self.result_filename + '.csv')
        if result_file.is_file() is False:
            result_file.touch()
            self.append_csv(result_file, self.metrics_header)

        self.append_csv(result_file, data)

    def append_csv(self, file_path, data):
        with open(file_path, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def perform_mission(self):
        self.get_logger().warning("No mission defined!!!")

    def perform_task(self, task_name, condition):
        self.get_logger().info('Starting {} task'.format(task_name))
        self.task_timer = self.create_rate(1)
        task_status = "completed"
        if self.abort_mission is False:
            self.request_task(task_name)
            while condition() is not True:
                if self.abort_mission is True:
                    task_status = "aborted"
                    break
                self.task_timer.sleep()
        else:
            task_status = "aborted"
        self.get_logger().info('Task {0} {1}'.format(task_name, task_status))
        self.cancel_task(task_name)
