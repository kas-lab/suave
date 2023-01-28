import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    mission_type = LaunchConfiguration('mission_type')
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')

    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path to save mission measured metrics'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='time_constrained_mission_results',
        description='Filename for the mission measured metrics'
    )

    time_limit_arg = DeclareLaunchArgument(
        'time_limit',
        default_value='300',
        description='Time limit for the mission (seconds)'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )

    mission_node = Node(
        package='suave_missions',
        executable=mission_type,
        name=mission_type,
        parameters=[{
            'result_path': result_path,
            'result_filename': result_filename,
            'time_limit': time_limit,
        }]
    )

    return LaunchDescription([
        mission_type_arg,
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        mission_node,
    ])