#!/usr/bin/env python
import sys
import rclpy

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from suave_missions.inspection_mission import InspectionMission


class MissionTimeConstrained(InspectionMission):
    def __init__(self, node_name='time_contrained_mission'):
        super().__init__(node_name)
        if self.result_filename == 'mission_results':
            self.result_filename = 'time_mission_results'

        self.declare_parameter('time_limit', 300)
        self.time_limit = self.get_parameter('time_limit').value

        self.time_monitor_timer = self.create_timer(
            0.5,
            self.time_monitor_cb,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.save_mission_results_cli = self.create_client(
            Empty,
            'mission_metrics/save',
            callback_group=MutuallyExclusiveCallbackGroup())

    def time_monitor_cb(self):
        if self.mission_start_time is not None:
            current_time = self.get_clock().now()
            elapsed_time = current_time - self.mission_start_time
            if elapsed_time.to_msg().sec >= self.time_limit:
                self.abort_mission = True
                self.call_service(
                    self.save_mission_results_cli, Empty.Request())
                self.time_monitor_timer.destroy()

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


def main():
    rclpy.init(args=sys.argv)

    mission_node = MissionTimeConstrained()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(mission_node)
    mt_executor.create_task(mission_node.perform_mission)
    mt_executor.spin()

    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
