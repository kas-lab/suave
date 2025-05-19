import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    silent = LaunchConfiguration('silent')
    silent_arg = DeclareLaunchArgument(
        'silent',
        default_value='false',
        description='Suppress all output (launch logs + node logs)'
    )
    def configure_logging(context, *args, **kwargs):
        if silent.perform(context) == 'true':
            import logging
            logging.getLogger().setLevel(logging.CRITICAL)
        return []
    
    result_filename = LaunchConfiguration('result_filename')
    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='random_results',
        description='Name of the results file'
    )
    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    pkg_suave_path = get_package_share_directory(
        'suave')

    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'task_bridge': 'False'}.items()
    )

    task_bridge_node = Node(
        package='suave_random',
        executable='task_bridge_random',
        parameters=[mission_config],
    )

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_node = Node(
        package='suave_missions',
        executable='time_constrained_mission',
        name='mission_node',
        parameters=[mission_config],
    )

    mission_metrics_node = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'random',
            'mission_name': 'suave',
            'result_filename': result_filename,
        }],
    )

    return LaunchDescription([
        result_filename_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        suave_launch,
        task_bridge_node,
        mission_node,
        mission_metrics_node
    ])
