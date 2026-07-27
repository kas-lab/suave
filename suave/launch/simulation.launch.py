# Copyright 2026 KAS Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the SUAVE simulator and vehicle support nodes."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Return the configurable SUAVE simulation launch description."""
    silent = LaunchConfiguration('silent')

    def configure_logging(context, *args, **kwargs):
        if silent.perform(context) == 'true':
            import logging
            logging.getLogger().setLevel(logging.ERROR)
        return []

    silent_arg = DeclareLaunchArgument(
        'silent',
        default_value='false',
        description='Suppress all output (launch logs + node logs)'
    )

    remaro_worlds_path = get_package_share_directory('remaro_worlds')
    min_pipes_launch_path = os.path.join(
        remaro_worlds_path, 'launch', 'small_min_pipes.launch.py')

    print_output = PythonExpression([
        '"log" if "',
        LaunchConfiguration('silent'),
        '" == "true" else "',
        LaunchConfiguration('print_output'),
        '"'
    ])
    print_output_arg = DeclareLaunchArgument(
        'print_output',
        default_value='screen',
        description='Whether to print output to terminal (screen/log)'
    )

    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    min_pipes_sim = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(min_pipes_launch_path),
        launch_arguments={
           'gui': gui,
           'print_output': print_output,
        }.items()
    )

    mavros_path = get_package_share_directory('mavros')
    mavros_launch_path = os.path.join(
        mavros_path, 'launch', 'node.launch')

    suave_path = get_package_share_directory('suave')
    mavros_config_default = os.path.join(
        suave_path,
        'config',
        'suave_mavros_apm_config.yaml')
    mavros_pluginlists_default = os.path.join(
        suave_path,
        'config',
        'suave_mavros_apm_pluginlists.yaml')

    fcu_url = LaunchConfiguration('fcu_url')
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://0.0.0.0:14550@14555',
        description='MAVROS FCU connection URL'
    )

    gcs_url = LaunchConfiguration('gcs_url')
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='Optional MAVROS GCS forwarding URL'
    )

    mavros_config_yaml = LaunchConfiguration('mavros_config_yaml')
    mavros_config_yaml_arg = DeclareLaunchArgument(
        'mavros_config_yaml',
        default_value=mavros_config_default,
        description='MAVROS parameter configuration YAML'
    )

    mavros_pluginlists_yaml = LaunchConfiguration('mavros_pluginlists_yaml')
    mavros_pluginlists_yaml_arg = DeclareLaunchArgument(
        'mavros_pluginlists_yaml',
        default_value=mavros_pluginlists_default,
        description='MAVROS plugin allowlist/denylist YAML'
    )

    mavros_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_path),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'tgt_system': '1',
            'tgt_component': '1',
            'config_yaml': mavros_config_yaml,
            'pluginlists_yaml': mavros_pluginlists_yaml,
            'log_output': print_output,
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false',
            'namespace': 'mavros',
            }.items()
        )

    bluerov2_ignition_path = get_package_share_directory('bluerov2_ignition')
    bluerov2_path = os.path.join(
        bluerov2_ignition_path, 'models', 'bluerov2')

    gz_pipe_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/min_pipes_pipeline/pose@geometry_msgs/msg/PoseArray@'
            'gz.msgs.Pose_V'],
        output=print_output,
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
        output=print_output,
        name='gz_bluerov_pose_bridge',
    )

    bluerov_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output=print_output,
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
        fcu_url_arg,
        gcs_url_arg,
        mavros_config_yaml_arg,
        mavros_pluginlists_yaml_arg,
        print_output_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        min_pipes_sim,
        bluerov_spawn,
        gz_pipe_pose_bridge,
        gz_bluerov_pose_bridge,
        mavros_node,
    ])
