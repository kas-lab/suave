import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    water_visibility_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='water_visibility_observer',
        name='water_visibility_observer',
    )

    pipeline_metacontrol_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='pipeline_metacontrol_node',
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
        metacontrol_launch,
        water_visibility_node,
        pipeline_metacontrol_node,
        mros2_system_modes_bridge_node,
        spiral_lc_node,
        follow_pipeline_lc,
        system_modes_launch,
    ])
