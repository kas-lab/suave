import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')
    mission_type = LaunchConfiguration('mission_type')
    adapt_manager = LaunchConfiguration('adapt_manager')


    
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
            'adapt_manager': adapt_manager,
        }]
    )

    goal_bridge_node = Node(
        package='suave_metacontrol',
        executable='metacontrol_goal_update',
        name='metacontrol_goal_update_node',
    )
    return LaunchDescription([
        adapt_manager_arg,
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        mission_type_arg,
        mission_node,
        goal_bridge_node,
    ])