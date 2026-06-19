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

"""Launch the random adaptation manager and SUAVE mission."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the random-manager launch description."""
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

    result_path = LaunchConfiguration('result_path')
    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path where to save the results'
    )

    result_filename = LaunchConfiguration('result_filename')
    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='random_results',
        description='Name of the results file'
    )

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_config = LaunchConfiguration('mission_config')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=mission_config_default,
        description='Mission config full path'
    )

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
        name='task_bridge',
        parameters=[mission_config],
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
            'result_path': result_path,
        }],
        condition=LaunchConfigurationEquals('result_filename', '')
    )

    mission_metrics_node_override = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'random',
            'mission_name': 'suave',
            'result_filename': result_filename,
            'result_path': result_path,
        }],
        condition=LaunchConfigurationNotEquals('result_filename', '')
    )

    return LaunchDescription([
        result_path_arg,
        result_filename_arg,
        silent_arg,
        mission_config_arg,
        OpaqueFunction(function=configure_logging),
        suave_launch,
        task_bridge_node,
        mission_node,
        mission_metrics_node,
        mission_metrics_node_override
    ])
