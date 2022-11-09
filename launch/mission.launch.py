import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_pipeline_inspection = get_package_share_directory(
        'pipeline_inspection')

    mavros_launch_path = os.path.join(
        pkg_pipeline_inspection, 'launch', 'mavros.launch.py')

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mavros_launch_path))

    follow_waypoint_node = Node(
        package='pipeline_inspection',
        executable='follow_waypoints.py',
        output='screen',
    )

    thruster_failure_node = Node(
        package='pipeline_inspection',
        executable='thruster_failures.py',
        output='screen',
    )

    return LaunchDescription([
        # mavros_launch,
        follow_waypoint_node,
        thruster_failure_node,
    ])
