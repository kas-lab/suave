import os
from datetime import datetime
from geometry_msgs.msg import Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool
from suave_missions.mission_planner import MissionPlanner


class InspectionMission(MissionPlanner):
    def __init__(self, node_name='inspection_mission'):
        super().__init__(node_name)
        self.declare_parameter('adaptation_manager', 'none')
        self.adaptation_manager = self.get_parameter(
            'adaptation_manager').value

        self.mission_name = 'inspection mission ' + str(
            self.adaptation_manager)
        self.metrics_header = [
            'mission_name',
            'datetime',
            'initial pos (x,y)',
            'time_mission (s)',
            'time_search (s)']

        # TODO: get this automatically
        self.initial_x = 0.0
        self.initial_y = 0.0

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

        self.gazebo_pos_sub = self.create_subscription(
            Pose,
            'model/bluerov2/pose',
            self.gazebo_pos_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.pipeline_detected_time = None
        self.first_gz_pose = True

    def perform_mission(self):
        self.get_logger().info("Pipeline inspection mission starting!!")
        self.timer = self.create_rate(1)

        while not self.status.armed:
            self.get_logger().info(
                'BlueROV is armed: {}'.format(self.status.armed))
            self.arm_motors(True)
            self.timer.sleep()

        guided_mode = 'GUIDED'
        while self.status.mode != guided_mode:
            self.get_logger().info(
                'BlueROV mode is: {} waiting for GUIDED mode'.format(
                 self.status.mode))
            self.set_mode(guided_mode)
            self.timer.sleep()

        self.mission_start_time = self.get_clock().now()

        self.perform_task('search_pipeline', lambda: self.pipeline_detected)
        self.perform_task('inspect_pipeline', lambda: self.pipeline_inspected)

        self.mission_completed_time = self.get_clock().now()
        self.exit_mission()

    def exit_mission(self):
        detection_time_delta = -1
        if self.pipeline_detected_time is not None:
            detection_time_delta = \
                self.pipeline_detected_time - self.mission_start_time
            detection_time_delta = detection_time_delta.to_msg().sec

        mission_time_delta = \
            self.mission_completed_time - self.mission_start_time

        self.get_logger().info(
            'Time elapsed to detect pipeline: {} seconds'.format(
                detection_time_delta))
        self.get_logger().info(
            'Time elapsed to complete mission: {} seconds'.format(
                mission_time_delta.to_msg().sec))

        mission_data = [
            self.mission_name,
            datetime.now().strftime("%b-%d-%Y-%H-%M-%S"),
            '({0}, {1})'.format(
                round(self.initial_x, 2), round(self.initial_y, 2)),
            mission_time_delta.to_msg().sec,
            detection_time_delta]

        self.save_metrics(mission_data)
        # TODO: make this a parameter
        os.system("touch ~/suave_ws/mission.done")

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

    def gazebo_pos_cb(self, msg):
        self.gazebo_pos = msg
        if self.first_gz_pose is True:
            self.first_gz_pose = False
            self.initial_x = msg.position.x
            self.initial_y = msg.position.y

        self.destroy_subscription(self.gazebo_pos_sub)
