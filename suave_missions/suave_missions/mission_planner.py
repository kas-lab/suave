import csv
from pathlib import Path
from rclpy.node import Node
from suave_msgs.srv import Task


class MissionPlanner(Node):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)

        self.task_request_service = self.create_client(
            Task, 'task/request')
        self.task_cancel_service = self.create_client(
            Task, 'task/cancel')

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')

        self.result_path = self.get_parameter('result_path').value
        self.result_filename = self.get_parameter('result_filename').value

        self.metrics_header = ['mission_name', 'datetime', 'metric']

        self.mission_start_time = None
        self.abort_mission = False

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
