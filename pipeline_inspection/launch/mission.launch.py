import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_pipeline_inspection = get_package_share_directory(
        'pipeline_inspection')

    mission_node = Node(
        package='pipeline_inspection',
        executable='mission.py',
        output='screen',
    )

    spiral_node = Node(
        package='pipeline_inspection',
        executable='spiral_mission.py',
        output='screen',
    )
    
    pipeline_node = Node(
        package='pipeline_inspection',
        executable='pipeline_node.py',
        output='screen',
    )

    follow_pipeline_node = Node(
        package='pipeline_inspection',
        executable='follow_pipeline.py',
        output='screen',
    )

    thruster_failure_node = Node(
        package='pipeline_inspection',
        executable='thruster_failures.py',
        output='screen',
    )

    return LaunchDescription([
        mission_node,
        spiral_node,
        pipeline_node,
        follow_pipeline_node,
        # thruster_failure_node,
    ])
