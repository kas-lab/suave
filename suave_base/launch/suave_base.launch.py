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

"""Launch the SUAVE managed system and mission metrics without a manager."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the base SUAVE launch description."""
    mission_config = LaunchConfiguration('mission_config')
    use_action_server = LaunchConfiguration('use_action_server')
    silent = LaunchConfiguration('silent')
    adaptation_manager = LaunchConfiguration('adaptation_manager')
    mission_type = LaunchConfiguration('mission_type')
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    arguments = [
        DeclareLaunchArgument(
            'mission_config',
            default_value=mission_config_default,
            description='Mission configuration file'),
        DeclareLaunchArgument(
            'use_action_server',
            default_value='false',
            description='Start managed behaviors through ROS action servers'),
        DeclareLaunchArgument(
            'silent',
            default_value='false',
            description='Suppress all output'),
        DeclareLaunchArgument(
            'adaptation_manager',
            default_value='',
            description='Adaptation manager label written to metrics'),
        DeclareLaunchArgument(
            'mission_type',
            default_value='time_constrained_mission',
            description='Mission label written to metrics'),
        DeclareLaunchArgument(
            'result_path',
            default_value='~/suave/results',
            description='Path where to save results'),
        DeclareLaunchArgument(
            'result_filename',
            default_value='',
            description='Name of the results file'),
    ]

    managed_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('suave'),
                'launch', 'suave.launch.py')),
        launch_arguments={
            'task_bridge': 'False',
            'mission_config': mission_config,
            'use_action_server': use_action_server,
            'silent': silent,
        }.items())

    mission_metrics_node = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': adaptation_manager,
            'mission_name': mission_type,
            'result_path': result_path,
        }],
        condition=LaunchConfigurationEquals('result_filename', ''))

    mission_metrics_node_filename = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': adaptation_manager,
            'mission_name': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
        }],
        condition=LaunchConfigurationNotEquals('result_filename', ''))

    return LaunchDescription([
        *arguments,
        managed_system,
        mission_metrics_node,
        mission_metrics_node_filename,
    ])
