import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    time_limit = LaunchConfiguration('time_limit')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    pkg_suave_path = get_package_share_directory(
        'suave')
    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

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
        package='suave_metacontrol',
        executable='task_bridge_random',
        parameters=[mission_config],
    )

    return LaunchDescription([
        suave_launch,
        task_bridge_node,
    ])
