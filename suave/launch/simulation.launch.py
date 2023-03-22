import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    remaro_worlds_path = get_package_share_directory('remaro_worlds')
    min_pipes_launch_path = os.path.join(
        remaro_worlds_path, 'launch', 'small_min_pipes.launch.py')

    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    min_pipes_sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(min_pipes_launch_path),
        launch_arguments={
           'gui': gui
        }.items()
    )

    mavros_path = get_package_share_directory('mavros')
    mavros_launch_path = os.path.join(
        mavros_path, 'launch', 'apm.launch')
    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            'fcu_url': 'udp://127.0.0.1:14551@14555',
            'gcs_url': 'udp://@localhost:14550',
            'system_id': '255',
            'component_id': '240',
            'target_system_id': '1',
            'target_component_id': '1'
            }.items()
        )

    bluerov2_ignition_path = get_package_share_directory('bluerov2_ignition')
    bluerov2_path = os.path.join(
        bluerov2_ignition_path, 'models', 'bluerov2')

    gz_pipe_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/min_pipes_pipeline/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'],
        output='screen',
        name='gz_pipe_pose_bridge',
    )

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-17.0',
        description='Initial x coordinate for bluerov2'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='2.0',
        description='Initial y coordinate for bluerov2'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='-18.5',
        description='Initial z coordinate for bluerov2'
    )

    gz_bluerov_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/bluerov2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'],
        output='screen',
        name='gz_bluerov_pose_bridge',
    )

    bluerov_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-v4',
            '-g',
            '-world', 'min_pipes',
            '-file', bluerov2_path,
            '-name', 'bluerov2',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', '0']
    )

    return LaunchDescription([
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        min_pipes_sim,
        bluerov_spawn,
        gz_pipe_pose_bridge,
        gz_bluerov_pose_bridge,
        mavros_node,
    ])
