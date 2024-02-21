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
        if req.task_name not in self.task_functions_dict:
            self.get_logger().error(
                'Request: {}'.format(req) +
                'Task requested is not available. ' +
                'Available tasks: {}'.format(self.task_functions_dict.keys())
            )
            response = Task.Response()
            response.success = False
            return response

        try:
            function_names = self.task_functions_dict[req.task_name]
            success = True
            for function in function_names:
                forward_request_result = forward_request(function)
                if type(forward_request_result) is bool:
                    success = forward_request_result and success
                else:
                    success = False
            response.success = success
        except Exception as e:
            self.get_logger().error(
                'Exception: {0}. Request: {1}'.format(e, req)
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
        if response.success is True and req.task_name in self.current_tasks:
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
        if cli.wait_for_service(timeout_sec=5.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()
