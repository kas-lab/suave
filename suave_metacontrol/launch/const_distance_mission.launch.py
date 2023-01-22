import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')

    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path to save mission measured metrics'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='const_distance_mission_results',
        description='Filename for the mission measured metrics'
    )

    mission_node = Node(
        package='suave_metacontrol',
        executable='const_dist_mission',
        name='const_dist_mission_node',
        parameters=[{
            'result_path': result_path,
            'result_filename': result_filename,
        }]
    )

    return LaunchDescription([
        result_path_arg,
        result_filename_arg,
        mission_node,
    ])
