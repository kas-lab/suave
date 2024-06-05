# Copyright 2024 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""module used to save metrics."""
import csv
import os
import sys
from datetime import datetime
from pathlib import Path
import statistics

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.msg import TransitionEvent

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



class MissionMetrics(Node):
    """ROS node used to save mission metrics.

    ROS params: `result_path`, `result_filename`, `adaptation_manager`,
    `mission_name`.

    """

    def __init__(self, node_name: str = 'suave_metrics'):
        super().__init__(node_name)

        self.declare_parameter('result_path', '~/suave/results')
        self.declare_parameter('result_filename', 'mission_results')
        self.declare_parameter('adaptation_manager', 'none')
        self.declare_parameter('mission_name', 'inspection')
        # self.declare_parameter('metrics_header', [''])
        self.declare_parameter(
            'water_visibiity_threshold', [3.25, 2.25, 1.25])
        self.declare_parameter(
            'expected_altitude', [3.0, 2.0, 1.0])

        self.declare_parameter(
            'battery_limit', 0.25)

        #: path where results must be saved
        self.result_path = self.get_parameter('result_path').value
        #: file name where results must be saved
        self.result_filename = self.get_parameter('result_filename').value
        #: adaptation manager name, e.g., none, random, metacontrol
        self.adaptation_manager = self.get_parameter(
            'adaptation_manager').value

        self.mission_name = self.get_parameter('mission_name').value
        self.mission_name += ' ' + self.adaptation_manager

        self.mission_start_time = None
        self.pipeline_detected_time = None

        # TODO: get this automatically
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.first_gz_pose = True

        self.distance_inspected = 0.0

        self.wrong_altitude = False
        self.thrusters_failed = False
        self.battery_low = False
        self.wrong_altitude_time = None
        self.thrusters_failed_time = None
        self.battery_low_time = None
        self.component_recovery_time = []
        self.wv_reaction_time = []
        self.battery_reaction_time = []

        self.measured_wv = None

        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.status_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_detected_sub = self.create_subscription(
            Bool,
            'pipeline/detected',
            self.pipeline_detected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_inspected_sub = self.create_subscription(
            Bool,
            'pipeline/inspected',
            self.pipeline_inspected_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.pipeline_distance_inspected_sub = self.create_subscription(
            Float32,
            'pipeline/distance_inspected',
            self.distance_inspected_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.gazebo_pos_sub = self.create_subscription(
            Pose,
            'model/bluerov2/pose',
            self.gazebo_pos_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
        )
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_cb,
            self.qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.f_maintain_motion_node_state_sub = self.create_subscription(
            TransitionEvent,
            '/f_maintain_motion_node/transition_event',
            self.maintain_motion_transition_cb,
            self.qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.generate_recharge_path_node_state_sub = self.create_subscription(
            TransitionEvent,
            '/generate_recharge_path_node/transition_event',
            self.generate_recharge_path_transition_cb,
            self.qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.param_change_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.param_change_cb,
            self.qos,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_spiral_altitude_cli = self.create_client(
            GetParameters,
            '/f_generate_search_path_node/get_parameters',
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.save_mission_results_srv = self.create_service(
            Empty,
            'mission_metrics/save',
            self.save_mission_results_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def status_cb(self, msg):
        if msg.mode == 'GUIDED':
            self.mission_start_time = self.get_clock().now()
            self.destroy_subscription(self.state_sub)

    def pipeline_detected_cb(self, msg):
        if msg.data is True:
            self.pipeline_detected_time = self.get_clock().now()
            self.destroy_subscription(self.pipeline_detected_sub)

    def pipeline_inspected_cb(self, msg):
        if msg.data is True:
            self.save_mission_results()
            self.destroy_subscription(self.pipeline_inspected_sub)

    def distance_inspected_cb(self, msg):
        self.distance_inspected = msg.data

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gz_pose is True:
            self.first_gz_pose = False
            self.initial_x = msg.position.x
            self.initial_y = msg.position.y
            self.destroy_subscription(self.gazebo_pos_sub)

    def diagnostics_cb(self, msg):
        time = Time.from_msg(msg.header.stamp)
        measurement_messages = [
            'qa status',
            'qa measurement',
            'ea status',
            'ea measurement',
            'attribute measurement']
        component_messages = [
            'component status', 'component']
        for diagnostic_status in msg.status:
            if diagnostic_status.message.lower() in measurement_messages:
                self.check_altitude(diagnostic_status, time)
                self.check_battery(diagnostic_status, time)
            if diagnostic_status.message.lower() in component_messages:
                thrusters_ok = self.check_thrusther(diagnostic_status, time)

    def check_altitude(self, diagnostic_status, time):
        for value in diagnostic_status.values:
            if value.key == 'water_visibility':
                self.measured_wv = float(value.value)
                altitude = self.get_spiral_altitude()
                if altitude is None:
                    return
                expected = self.get_expected_spiral_altitude(value.value)
                correct_altitude =  altitude == expected
                if self.battery_low is False and self.wrong_altitude is False \
                 and correct_altitude is False:
                    self.wrong_altitude = True
                    self.wrong_altitude_time = time
                return

    def get_spiral_altitude(self):
        req = GetParameters.Request(names=['spiral_altitude'])
        res = self.call_service(self.get_spiral_altitude_cli, req)
        return res.values[0].double_value if res is not None and len(res.values) > 0 else None

    def get_expected_spiral_altitude(self, measured_wv):
        wv_threshold = self.get_parameter('water_visibiity_threshold').value
        expected_altitude = self.get_parameter('expected_altitude').value
        for threshold, expected in zip(wv_threshold, expected_altitude):
            if float(measured_wv) >= threshold:
                return expected
        return None

    def check_thrusther(self, diagnostic_status, time):
        for value in diagnostic_status.values:
            if value.key.startswith("c_thruster_") and value.value=="FALSE":
                if self.thrusters_failed is False:
                    self.thrusters_failed = True
                    self.thrusters_failed_time = time
                return

    def check_battery(self, diagnostic_status, time):
        for value in diagnostic_status.values:
            if value.key == 'battery_level':
                battery_limit = self.get_parameter('battery_limit').value
                if self.battery_low is False and float(value.value) < battery_limit:
                    self.battery_low = True
                    self.battery_low_time = time
                    self.wrong_altitude = False
                if self.battery_low is True and float(value.value) > battery_limit:
                    self.battery_low = False
                return

    def maintain_motion_transition_cb(self, msg):
        if msg.goal_state.label == "active" and self.thrusters_failed is True:
            reaction_time = self.get_clock().now() - self.thrusters_failed_time
            reaction_time = reaction_time.nanoseconds * 1e-9
            self.component_recovery_time.append(reaction_time)
            self.thrusters_failed = False
            self.get_logger().info(
                'Thruster failure reaction time: {} seconds'.format(reaction_time))

    def generate_recharge_path_transition_cb(self, msg):
        if msg.goal_state.label == "active" and self.battery_low is True:
            reaction_time = self.get_clock().now() - self.battery_low_time
            reaction_time = reaction_time.nanoseconds * 1e-9
            self.battery_reaction_time.append(reaction_time)
            # self.battery_low = False
            self.get_logger().info(
                'Battery drop reaction time: {} seconds'.format(reaction_time))

    def param_change_cb(self, msg):
        time = self.get_clock().now()
        if msg.node == "/f_generate_search_path_node"  \
         and self.wrong_altitude is True:
            expected_altitude = self.get_expected_spiral_altitude(
                self.measured_wv)
            for param in msg.changed_parameters:
                if param.name == "spiral_altitude" and param.value.double_value == expected_altitude:
                    reaction_time =  time - self.wrong_altitude_time
                    reaction_time = reaction_time.nanoseconds * 1e-9
                    self.wv_reaction_time.append(
                        reaction_time)
                    self.wrong_altitude = False
                    self.get_logger().info(
                        'Water visibility change reaction time: {0} seconds'.format(reaction_time))
                    return

    def save_mission_results_cb(
         self, req: Empty.Request, res: Empty.Response) -> None:
        self.save_mission_results()
        self.destroy_subscription(self.gazebo_pos_sub)
        self.destroy_subscription(self.pipeline_detected_sub)
        self.destroy_subscription(self.pipeline_inspected_sub)
        self.destroy_subscription(self.pipeline_distance_inspected_sub)
        return res

    def save_mission_results(self) -> None:
        """Process mission data and save it into a .csv file."""
        mission_time_delta = \
            self.get_clock().now() - self.mission_start_time

        pipeline_detected = False
        detection_time_delta = mission_time_delta.to_msg().sec
        if self.pipeline_detected_time is not None:
            detection_time_delta = \
                self.pipeline_detected_time - self.mission_start_time
            detection_time_delta = detection_time_delta.to_msg().sec
            pipeline_detected = True

        self.get_logger().info(
            'Time elapsed to detect pipeline: {} seconds'.format(
                detection_time_delta))
        self.get_logger().info(
            'Distance inspected: {} m'.format(
                self.distance_inspected))
        self.get_logger().info(
            'Time elapsed to complete mission: {} seconds'.format(
                mission_time_delta.to_msg().sec))

        mission_header = [
            'mission name',
            'datetime',
            'initial pos (x,y)',
            'mission duration (s)',
            'pipeline found',
            'time searching pipeline (s)',
            'distance inspected (m)',
            'mean reaction time (s)',
        ]

        date = datetime.now().strftime("%d-%b-%Y-%H-%M-%S")
        mean_reaction_time = 0.0
        try:
            mean_reaction_time = statistics.fmean(
                self.component_recovery_time +
                self.wv_reaction_time +
                self.battery_reaction_time)
        except statistics.StatisticsError as e:
            pass
        mission_data = [
            self.mission_name,
            date,
            '({0}, {1})'.format(
                round(self.initial_x, 2), round(self.initial_y, 2)),
            mission_time_delta.to_msg().sec,
            pipeline_detected,
            detection_time_delta,
            self.distance_inspected,
            mean_reaction_time
        ]

        self.save_metrics(
            self.result_path,
            self.result_filename,
            mission_header,
            mission_data)

        component_file = self.result_filename +'_component_recovery_time'
        for t in self.component_recovery_time:
            self.save_metrics(
                self.result_path,
                component_file,
                ['mission_name', 'datetime', 'reaction time (s)'],
                [self.mission_name, date, t]
            )

        wv_file = self.result_filename +'_wv_reaction_time'
        for t in self.wv_reaction_time:
            self.save_metrics(
                self.result_path,
                wv_file,
                ['mission_name', 'datetime', 'reaction time (s)'],
                [self.mission_name, date, t]
            )

        battery_file = self.result_filename +'_battery_reaction_time'
        for t in self.battery_reaction_time:
            self.save_metrics(
                self.result_path,
                battery_file,
                ['mission_name', 'datetime', 'reaction time (s)'],
                [self.mission_name, date, t]
            )

        os.system("touch /tmp/mission.done")

    def save_metrics(
        self,
        path: str,
        filename: str,
        header: list[str],
        data: list[str | float | int]) -> None:
        """Save data into a .csv file.

        Create folder if `result_path` folder does not exist.
        Create file if `result_file` does not exist and insert header.
        Append data if `result_file` exist.

        :param data: array with data to be saved
        """
        result_path = Path(path).expanduser()

        if result_path.is_dir() is False:
            result_path.mkdir(parents=True)

        result_file = result_path / (filename + '.csv')
        if result_file.is_file() is False:
            result_file.touch()
            self.append_csv(result_file, header)

        self.append_csv(result_file, data)

    def append_csv(self, file_path: str,
                   data: list[str | float | int]) -> None:
        """Append array to .csv file.

        :param file_path: file path
        :param data: data to be saved
        """
        with open(file_path, 'a') as file:
            writer = csv.writer(file)
            writer.writerow(data)

    def call_service(self, cli, request):
        if cli.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().error(
                'service not available {}'.format(cli.srv_name))
            return None
        future = cli.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=1.0)
        if future.done() is False:
            self.get_logger().error(
                'Future not completed {}'.format(cli.srv_name))
            return None
        return future.result()

def main():
    rclpy.init(args=sys.argv)

    metrics_node = MissionMetrics()

    mt_executor = rclpy.executors.MultiThreadedExecutor()
    mt_executor.add_node(metrics_node)
    mt_executor.spin()

    metrics_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
