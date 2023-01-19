import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')
    f_generate_search_path_mode = LaunchConfiguration(
        'f_generate_search_path_mode')
    f_inspect_pipeline_mode = LaunchConfiguration('f_inspect_pipeline_mode')

    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/pipeline_inspection/results',
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

    f_generate_search_path_mode_arg = DeclareLaunchArgument(
        'f_generate_search_path_mode',
        default_value='fd_spiral_low',
        description='Desired mode for f_generate_search_path_mode'
    )

    f_inspect_pipeline_mode_arg = DeclareLaunchArgument(
        'f_inspect_pipeline_mode',
        default_value='fd_inspect_pipeline',
        description='Desired mode for f_inspect_pipeline_mode'
    )

    mission_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='time_constrained_mission_no_adapt',
        name='time_constrained_mission_no_adapt_node',
        parameters=[{
            'result_path': result_path,
            'result_filename': result_filename,
            'time_limit': time_limit,
            'f_generate_search_path_mode': f_generate_search_path_mode,
            'f_inspect_pipeline_mode': f_inspect_pipeline_mode,
        }]
    )

    return LaunchDescription([
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        f_generate_search_path_mode_arg,
        f_inspect_pipeline_mode_arg,
        mission_node,
    ])
