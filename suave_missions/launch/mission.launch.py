import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')
    adapt_manager = LaunchConfiguration('adapt_manager')
    f_generate_search_path_mode = LaunchConfiguration('f_generate_search_path_mode')
    f_inspect_pipeline_mode = LaunchConfiguration('f_inspect_pipeline_mode')
    mission_type = LaunchConfiguration('mission_type')
    adapt_period = LaunchConfiguration("adaptation_period")

    adapt_period_arg = DeclareLaunchArgument(
        'adaptation_period',
        default_value='15',
        description='How often the random adaptation happens every x (seconds)'
    )

    adapt_manager_arg = DeclareLaunchArgument(
        'adapt_manager',
        default_value='none',
        description='Which adaptation manager is in charge, none/metacontrol/random'
    )

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
        description='IF a time constrained mission then time limit for the mission (seconds)'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
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

    pkg_suave_path = get_package_share_directory(
        'suave_missions')

    print("SOMETHING\n\n\n\n")
    # mission_launch_str = [adapt_manager, '_mission.launch.py']
    # mission_launch_path = os.path.join(
    # pkg_suave_path,
    # 'launch',
    # ''.join(mission_launch_str)
    # )

    mission_launch_path = [PathJoinSubstitution([pkg_suave_path, 'launch', adapt_manager]), TextSubstitution(text='_mission.launch.py')]
    LogInfo(msg=mission_launch_path)


    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission_launch_path),
        launch_arguments={
            'mission_type': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
            'time_limit': time_limit,
            'f_generate_search_path_mode': f_generate_search_path_mode,
            'f_inspect_pipeline_mode': f_inspect_pipeline_mode,
            'adapt_period': adapt_period,
        }.items())

    return LaunchDescription([
        adapt_manager_arg,
        adapt_period_arg,
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        mission_type_arg,
        f_generate_search_path_mode_arg,
        f_inspect_pipeline_mode_arg,
        mission_launch,
    ])