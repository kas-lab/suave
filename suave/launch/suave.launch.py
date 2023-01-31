import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    water_visibility_node = Node(
        package='suave',
        executable='water_visibility_observer',
        name='water_visibility_observer_node',
        parameters=[mission_config],
    )

    pipeline_detection_wv_node = Node(
        package='suave',
        executable='pipeline_detection_wv',
    )

    thruster_monitor_node = Node(
        package='suave',
        executable='thruster_monitor',
        name='thruster_monitor',
        parameters=[mission_config]
    )

    spiral_search_node = Node(
        package='suave',
        executable='spiral_search',
        output='screen'
    )

    follow_pipeline_node = Node(
        package='suave',
        executable='follow_pipeline',
        output='screen',
    )

    pkg_suave_path = get_package_share_directory('suave')

    system_modes_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'system_modes.launch.py')

    system_modes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_modes_launch_path))

    return LaunchDescription([
        water_visibility_node,
        pipeline_detection_wv_node,
        thruster_monitor_node,
        spiral_search_node,
        follow_pipeline_node,
        system_modes_launch,
    ])
