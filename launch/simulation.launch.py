import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    remaro_worlds_path = get_package_share_directory('remaro_worlds')
    min_pipes_launch_path = os.path.join(
        remaro_worlds_path, 'launch', 'min_pipes.launch.py')

    min_pipes_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(min_pipes_launch_path))

    bluerov2_ignition_path = get_package_share_directory('bluerov2_ignition')
    bluerov2_path = os.path.join(
        bluerov2_ignition_path, 'models', 'bluerov2')

    gz_pipe_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/min_pipes_pipeline/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'],
        output='screen'
    )

    gz_bluerov_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/bluerov2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose'],
        output='screen'
    )

    # TODO: Pass x, y, z, R, P and Y as parameter
    bluerov_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'min_pipes',
            '-file', bluerov2_path,
            '-name', 'bluerov2',
            '-x', '0',
            '-y', '10',
            '-z', '-17.5',
            '-Y', '0']
    )

    return LaunchDescription([
        min_pipes_sim,
        bluerov_spawn,
        gz_pipe_pose_bridge,
        gz_bluerov_pose_bridge,
    ])
