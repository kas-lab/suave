import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_suave_path = get_package_share_directory(
        'suave')
    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'metacontrol.launch.py')

    metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_launch_path))

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    recover_thrusters_node = Node(
        package='suave',
        executable='recover_thrusters'
    )

    water_visibility_period = LaunchConfiguration('water_visibility_period')
    water_visibility_min = LaunchConfiguration('water_visibility_min')
    water_visibility_max = LaunchConfiguration('water_visibility_max')
    water_visibility_sec_shift = LaunchConfiguration(
        'water_visibility_sec_shift')

    water_visibility_period_arg = DeclareLaunchArgument(
        'water_visibility_period',
        default_value='100',
        description='Water visibility period in seconds'
    )

    water_visibility_min_arg = DeclareLaunchArgument(
        'water_visibility_min',
        default_value='1.25',
        description='Minimum value for water visibility'
    )

    water_visibility_max_arg = DeclareLaunchArgument(
        'water_visibility_max',
        default_value='3.75',
        description='Maximum value for water visibility'
    )

    water_visibility_sec_shift_arg = DeclareLaunchArgument(
        'water_visibility_sec_shift',
        default_value='0.0',
        description='Water visibility seconds shift to left'
    )

    thruster_events = LaunchConfiguration('thruster_events')
    thruster_events_arg = DeclareLaunchArgument(
        'thruster_events',
        default_value=str(['(1, failure,30)', '(2, failure,30)']),
        description='(thrusterN, failure/recovery, delta time in seconds ),' +
        ' e.g. (1, failure, 50)'
    )

    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'water_visibility_period': water_visibility_period,
            'water_visibility_min': water_visibility_min,
            'water_visibility_max': water_visibility_max,
            'water_visibility_sec_shift': water_visibility_sec_shift,
            'thruster_events': thruster_events,
        }.items())

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    return LaunchDescription([
        water_visibility_period_arg,
        water_visibility_min_arg,
        water_visibility_max_arg,
        water_visibility_sec_shift_arg,
        thruster_events_arg,
        suave_launch,
        metacontrol_launch,
        mros2_system_modes_bridge_node,
        recover_thrusters_node,
    ])
