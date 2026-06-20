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

"""Compose a SUAVE mission with the selected adaptation manager."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from launch_ros.actions import Node


def _launch_file(package_name, filename):
    """Return the installed path for a package launch file."""
    return os.path.join(
        get_package_share_directory(package_name), 'launch', filename)


def generate_launch_description():
    """Return the top-level configurable mission launch description."""
    adaptation_manager = LaunchConfiguration('adaptation_manager')
    mission_type = LaunchConfiguration('mission_type')
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    mission_config = LaunchConfiguration('mission_config')
    battery_constraint = LaunchConfiguration('battery_constraint')
    battery_constraint_value = LaunchConfiguration('battery_constraint_value')
    reasoning_time_filename = LaunchConfiguration(
        'mc_reasoning_time_filename')
    use_action_server = LaunchConfiguration('use_action_server')
    bt_executable = LaunchConfiguration('bt_executable')
    silent = LaunchConfiguration('silent')

    mission_config_default = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')

    arguments = [
        DeclareLaunchArgument(
            'adaptation_manager',
            default_value='none',
            choices=['none', 'metacontrol', 'random', 'bt'],
            description='Adaptation manager in charge'),
        DeclareLaunchArgument(
            'mission_type',
            default_value='time_constrained_mission',
            description='Mission label written to metrics'),
        DeclareLaunchArgument(
            'result_path',
            default_value='~/suave/results',
            description='Path where to save the results'),
        DeclareLaunchArgument(
            'result_filename',
            default_value='',
            description='Name of the results file'),
        DeclareLaunchArgument(
            'mission_config',
            default_value=mission_config_default,
            description='Mission configuration file'),
        DeclareLaunchArgument(
            'battery_constraint',
            default_value='False',
            description='Whether to enforce the battery constraint'),
        DeclareLaunchArgument(
            'battery_constraint_value',
            default_value='0.2',
            description='Battery constraint value'),
        DeclareLaunchArgument(
            'mc_reasoning_time_filename',
            default_value='metacontrol_reasoning_time',
            description='Metacontrol reasoning-time filename'),
        DeclareLaunchArgument(
            'use_action_server',
            default_value='false',
            description='Start BT-managed behaviors through action servers'),
        DeclareLaunchArgument(
            'bt_executable',
            default_value='suave_bt',
            choices=['suave_bt', 'suave_bt_extended'],
            description='Behavior Tree executable'),
        DeclareLaunchArgument(
            'silent',
            default_value='false',
            description='Suppress managed-system output'),
    ]

    task_bridge = PythonExpression([
        "'True' if '", adaptation_manager, "' == 'none' else 'False'"])
    managed_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('suave', 'suave.launch.py')),
        launch_arguments={
            'task_bridge': task_bridge,
            'mission_config': mission_config,
            'use_action_server': use_action_server,
            'silent': silent,
        }.items())

    mission_node = Node(
        package='suave_missions',
        executable='time_constrained_mission',
        name='mission_node',
        parameters=[mission_config, {
            'battery_constraint': battery_constraint,
            'battery_constraint_value': battery_constraint_value,
        }],
        condition=LaunchConfigurationNotEquals('adaptation_manager', 'bt'))

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
    mission_metrics_node_override = Node(
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

    manager_launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file(
                'suave_none', 'suave_none.launch.py')),
            condition=LaunchConfigurationEquals(
                'adaptation_manager', 'none')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file(
                'suave_random', 'suave_random.launch.py')),
            launch_arguments={'mission_config': mission_config}.items(),
            condition=LaunchConfigurationEquals(
                'adaptation_manager', 'random')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file(
                'suave_metacontrol', 'suave_metacontrol.launch.py')),
            launch_arguments={
                'reasoning_time_filename': reasoning_time_filename,
                'result_path': result_path,
            }.items(),
            condition=LaunchConfigurationEquals(
                'adaptation_manager', 'metacontrol')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_file(
                'suave_bt', 'suave_bt.launch.py')),
            launch_arguments={
                'mission_config': mission_config,
                'use_action_server': use_action_server,
                'bt_executable': bt_executable,
            }.items(),
            condition=LaunchConfigurationEquals('adaptation_manager', 'bt')),
    ]

    return LaunchDescription([
        *arguments,
        managed_system,
        mission_node,
        mission_metrics_node,
        mission_metrics_node_override,
        *manager_launches,
    ])
