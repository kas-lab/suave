import os
from datetime import datetime
from suave_missions.mission_planner import MissionPlanner


class InspectionMission(MissionPlanner):
    def __init__(self, node_name='inspection_mission_node'):
        super().__init__(node_name)
        self.mission_name = 'inspection mission ' + str(
            self.adaptation_manager)
        self.metrics_header = [
            'mission_name',
            'datetime',
            'initial pos (x,y)',
            'time_mission (s)',
            'time_search (s)']

        # TODO: get this automatically
        self.initial_x = 1.0
        self.initial_y = 1.0

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
