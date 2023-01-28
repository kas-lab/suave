import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    f_generate_search_path_mode = LaunchConfiguration(
        'f_generate_search_path_mode')
    f_follow_pipeline_mode = LaunchConfiguration('f_follow_pipeline_mode')

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

    f_generate_search_path_mode_arg = DeclareLaunchArgument(
        'f_generate_search_path_mode',
        default_value='fd_spiral_low',
        description='Desired mode for f_generate_search_path_mode'
    )

    f_follow_pipeline_mode_arg = DeclareLaunchArgument(
        'f_follow_pipeline_mode',
        default_value='fd_follow_pipeline',
        description='Desired mode for f_follow_pipeline_mode'
    )

    mission_node = Node(
        package='suave',
        executable='const_dist_mission_no_adapt',
        name='const_dist_mission_no_adapt_node',
        parameters=[{
            'result_path': result_path,
            'result_filename': result_filename,
            'f_generate_search_path_mode': f_generate_search_path_mode,
            'f_follow_pipeline_mode': f_follow_pipeline_mode,
        }]
    )

    return LaunchDescription([
        result_path_arg,
        result_filename_arg,
        f_generate_search_path_mode_arg,
        f_follow_pipeline_mode_arg,
        mission_node,
    ])
