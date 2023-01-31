import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    remaro_worlds_path = get_package_share_directory('remaro_worlds')
    min_pipes_launch_path = os.path.join(
        remaro_worlds_path, 'launch', 'small_min_pipes.launch.py')

    min_pipes_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(min_pipes_launch_path))

    suave_path = get_package_share_directory(
        'suave')

    mavros_launch_path = os.path.join(
        suave_path, 'launch', 'mavros.launch.py')
    mavros_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mavros_launch_path))

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
    gui = LaunchConfiguration('gui')

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='-g',
        description='Run gazebo with a gui (-g) or without (-s)'
    )


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
        mavros_node
    ])
