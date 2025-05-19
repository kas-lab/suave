# Copyright 2024 Gustavo Rezende Silva
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
    
    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')
    mission_config = LaunchConfiguration('mission_config')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='suave',
        description='Mission name for logging'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=mission_config_default,
        description='Mission configuration file'
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

    mission_metrics_node = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'bt',
            'mission_name': mission_type,
        }],
        condition=LaunchConfigurationEquals('result_filename', '')
    )

    mission_metrics_node_override = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'bt',
            'mission_name': mission_type,
            'result_filename': result_filename,
        }],
        condition=LaunchConfigurationNotEquals('result_filename', '')
    )

    suave_bt_node = Node(
        package='suave_bt',
        executable='suave_bt',
        parameters=[mission_config]
    )

    return LaunchDescription([
        mission_type_arg,
        result_filename_arg,
        mission_config_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        suave_launch,
        mission_metrics_node,
        mission_metrics_node_override,
        suave_bt_node
    ])
