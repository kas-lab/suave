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
    time_limit = LaunchConfiguration('time_limit')

    recover_thrusters_node = Node(
        package='suave',
        executable='recover_thrusters'
    )

    adapt_period_arg = DeclareLaunchArgument(
        'adaptation_period',
        default_value='15',
        description='How often the random adaptation happens every x (seconds)'
    )

    adapt_period = LaunchConfiguration("adaptation_period")

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    random_reasoner_node = Node(
        package='suave_metacontrol',
        executable='random_reasoner',
        name='random_reasoner_node',
        parameters=[mission_config]
    )

    return LaunchDescription([
        adapt_period_arg,
        random_reasoner_node,
        recover_thrusters_node,
    ])
