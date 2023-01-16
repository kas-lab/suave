import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_pipeline_inspection_path = get_package_share_directory(
        'pipeline_inspection')
    pkg_pipeline_inspection_metacontrol_path = get_package_share_directory(
        'pipeline_inspection_metacontrol')

    metacontrol_launch_path = os.path.join(
        pkg_pipeline_inspection_metacontrol_path,
        'launch',
        'metacontrol.launch.py')
    system_modes_launch_path = os.path.join(
        pkg_pipeline_inspection_metacontrol_path,
        'launch',
        'system_modes.launch.py')

    metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_launch_path))
    system_modes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_modes_launch_path))

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

    water_visibility_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='water_visibility_observer',
        name='water_visibility_observer',
        parameters=[{
            'water_visibility_period': water_visibility_period,
            'water_visibility_min': water_visibility_min,
            'water_visibility_max': water_visibility_max,
            'water_visibility_sec_shift': water_visibility_sec_shift,
        }],
    )

    pipeline_metacontrol_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='pipeline_metacontrol_node',
        output='screen'
    )

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    spiral_lc_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='spiral_lc_node',
        output='screen'
    )

    follow_pipeline_lc = Node(
        package='pipeline_inspection_metacontrol',
        executable='follow_pipeline_lc',
        output='screen',
    )

    return LaunchDescription([
        water_visibility_period_arg,
        water_visibility_min_arg,
        water_visibility_max_arg,
        water_visibility_sec_shift_arg,
        metacontrol_launch,
        water_visibility_node,
        pipeline_metacontrol_node,
        mros2_system_modes_bridge_node,
        spiral_lc_node,
        follow_pipeline_lc,
        system_modes_launch,
    ])
