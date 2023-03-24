from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from suave_msgs.srv import Task


class TaskBridge(Node):
    def __init__(self):
        super().__init__('adpatation_goal_bridge')

        self.task_cb_group = MutuallyExclusiveCallbackGroup()
        self.task_request_service = self.create_service(
            Task,
            'task/request',
            self.task_request_cb,
            callback_group=self.task_cb_group
        )

        self.task_cancel_service = self.create_service(
            Task,
            'task/cancel',
            self.task_cancel_cb,
            callback_group=self.task_cb_group
        )

        self.current_tasks = set()
        self.task_functions_dict = dict()

    def task_request(self, req, forward_request):
        response = Task.Response()
        try:
            function_names = self.task_functions_dict[req.task_name]
            success = True
            for function in function_names:
                success = forward_request(function) and success
            response.success = success
        except Exception as e:
            self.get_logger().error(
                'Exception: {}. '.format(e) +
                'Probably requested wrong task name. ' +
                'Available tasks: {}'.format(self.task_functions_dict.keys())
            )
            response = Task.Response()
            response.success = False

        return response

    def task_request_cb(self, req, res):
        response = self.task_request(req, self.forward_task_request)
        if response.success is True:
            self.current_tasks.add(req.task_name)
        return response

    def task_cancel_cb(self, req, response):
        response = self.task_request(req, self.forward_task_cancel_request)
        if response.success is True:
            self.current_tasks.remove(req.task_name)
        return response

    def forward_task_request(self, function):
        self.get_logger().info("forward_task_request method not defined")
        return False

    def forward_task_cancel_request(self, function):
        self.get_logger().info(
            "forward_task_cancel_request method not defined")
        return False

    def call_service(self, cli, request):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = cli.call_async(request)
        while self.executor.spin_until_future_complete(
                future, timeout_sec=1.0):
            self.get_logger().info("Waiting for future to complete")
        return future.result()
