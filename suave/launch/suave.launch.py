import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    task_bridge = LaunchConfiguration('task_bridge')
    task_bridge_arg = DeclareLaunchArgument(
        'task_bridge',
        default_value='True',
        description='Indicates if task_bridge should be launched [True/False]'
    )

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

    recover_thrusters_node = Node(
        package='suave',
        executable='recover_thrusters'
    )

    task_bridge_node = Node(
        package='suave',
        executable='task_bridge_none',
        condition=LaunchConfigurationEquals('task_bridge', 'True')
    )

    pkg_suave_path = get_package_share_directory('suave')

    system_modes_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'system_modes.launch.py')

    system_modes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_modes_launch_path))

    return LaunchDescription([
        task_bridge_arg,
        water_visibility_node,
        pipeline_detection_wv_node,
        thruster_monitor_node,
        spiral_search_node,
        follow_pipeline_node,
        recover_thrusters_node,
        task_bridge_node,
        system_modes_launch,
    ])
