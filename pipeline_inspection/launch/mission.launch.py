import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    mission_node = Node(
        package='pipeline_inspection',
        executable='mission',
        output='screen',
    )

    spiral_node = LifecycleNode(
        package='pipeline_inspection_metacontrol',
        executable='spiral_lc_node',
        name='spiral_lc_node',
        namespace='',
        output='screen',
        parameters=[
            {'spiral_width': 1.0}
        ],
    )

    # spiral_node = Node(
        # package='metacontrol',
        # executable='spiral_lc_node',
        # name='spiral_lc_node',
        # output='screen',
        # parameters=[
            # {'spiral_width': 1.0}
        # ],
    # )
    
    pipeline_node = Node(
        package='pipeline_inspection',
        executable='pipeline_node',
        output='screen',
    )

    follow_pipeline_node = Node(
        package='pipeline_inspection',
        executable='follow_pipeline',
        output='screen',
    )

    thruster_failure_node = Node(
        package='pipeline_inspection',
        executable='thruster_failures',
        output='screen',
    )

    return LaunchDescription([
        mission_node,
        spiral_node,
        pipeline_node,
        follow_pipeline_node,
        # thruster_failure_node,
    ])
