import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adaptation_manager = LaunchConfiguration('adaptation_manager')
    mission_type = LaunchConfiguration('mission_type')

    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge, none/metacontrol/random')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
        mission_node,
        metacontrol_launch,
        suave_launch,
    ])
