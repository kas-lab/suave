import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    adapt_manager = LaunchConfiguration('adapt_manager')
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')
    f_generate_search_path_mode = LaunchConfiguration(
        'f_generate_search_path_mode')
    f_inspect_pipeline_mode = LaunchConfiguration('f_inspect_pipeline_mode')
    mission_type = LaunchConfiguration('mission_type')

    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path to save mission measured metrics'
    )

    adapt_manager_arg = DeclareLaunchArgument(
        'adapt_manager',
        default_value='none',
        description='Which adaptation manager is in charge, none/metacontrol/random'
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
        package='suave_missions',
        executable=mission_type,
        name=mission_type,
        parameters=[{
            'result_path': result_path,
            'result_filename': result_filename,
            'time_limit': time_limit,
            'f_generate_search_path_mode': f_generate_search_path_mode,
            'f_inspect_pipeline_mode': f_inspect_pipeline_mode,
            'adapt_manager': adapt_manager,
        }]
    )


    return LaunchDescription([
        adapt_manager_arg,
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        f_generate_search_path_mode_arg,
        f_inspect_pipeline_mode_arg,
        mission_node,
    ])
