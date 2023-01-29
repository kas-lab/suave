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

    metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'metacontrol.launch.py')

    metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_launch_path))

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    recover_thrusters_node = Node(
        package='suave',
        executable='recover_thrusters'
    )

    thruster_events = LaunchConfiguration('thruster_events')
    thruster_events_arg = DeclareLaunchArgument(
        'thruster_events',
        default_value=str(['(1, failure,30)', '(2, failure,30)']),
        description='(thrusterN, failure/recovery, delta time in seconds ),' +
        ' e.g. (1, failure, 50)'
    )

    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'thruster_events': thruster_events,
        }.items())

    mros2_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    
    goal_bride_node = Node(
        package='suave_metacontrol',
        executable='metacontrol_goal_update',
        name='metacontrol_goal_update_node',
    )

    return LaunchDescription([
        thruster_events_arg,
        suave_launch,
        metacontrol_launch,
        mros2_system_modes_bridge_node,
        recover_thrusters_node,
        goal_bride_node,
    ])
