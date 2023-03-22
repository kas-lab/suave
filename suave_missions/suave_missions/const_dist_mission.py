import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from suave_missions.inspection_mission import InspectionMission


def main():

    rclpy.init(args=sys.argv)

    mission_node = InspectionMission()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(mission_node)
    mt_executor.create_task(mission_node.perform_mission)
    mt_executor.spin()

    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
