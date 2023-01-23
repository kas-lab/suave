import rclpy

from mros2_reasoner.ros_reasoner import RosReasoner
from rclpy.executors import MultiThreadedExecutor

# TODO: I don't it is a good pratice to import something from tomasys
from mros2_reasoner.tomasys import get_current_function_design
from mros2_reasoner.tomasys import get_measured_qa


class SuaveReasoner(RosReasoner):
    def __init__(self):
        super().__init__()

    def analyze(self):
        objectives_in_error = []
        try:
            objectives_in_error = super().analyze()
        except Exception as err:
            self.logger.info("In Analyze, exception returned: {}".format(err))
        try:
            objectives = self.search_objectives()
            for objective in objectives:
                if objective not in objectives_in_error and \
                 str(objective.typeF.name) == "f_generate_search_path":
                    measured_water_visibility = get_measured_qa(
                        'water_visibility', self.tomasys)
                    current_fd = get_current_function_design(
                        objective, self.tomasys)
                    if current_fd is not None:
                        for qa in current_fd.hasQAestimation:
                            if str(qa.isQAtype.name) == 'water_visibility' \
                             and (qa.hasValue - measured_water_visibility) < 0:
                                objectives_in_error.append(objective)
                                objective.o_status = "IN_ERROR_NFR"
        except Exception as err:
            self.logger.info(
                "In Custom Analyze, exception returned: {}".format(err))
        return objectives_in_error


def main(args=None):

    rclpy.init(args=args)

    pipeline_inspection_reasoner = SuaveReasoner()

    mt_executor = MultiThreadedExecutor()

    if pipeline_inspection_reasoner.is_initialized is not True:
        pipeline_inspection_reasoner.get_logger().info(
            "There was an error in the reasoner initialization")
        return

    # Spin until the process in terminated
    rclpy.spin(pipeline_inspection_reasoner, executor=mt_executor)
    ros_reasoner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
