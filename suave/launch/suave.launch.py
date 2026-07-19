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

"""Launch the managed SUAVE system and its supporting nodes."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    """Return the managed-system launch description."""
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

    task_bridge_arg = DeclareLaunchArgument(
        'task_bridge',
        default_value='True',
        description=(
            'Indicates whether task_bridge should be launched [True/False]')
    )

    system_modes_arg = DeclareLaunchArgument(
        'system_modes',
        default_value='True',
        description=(
            'Indicates whether system_modes should be launched [True/False]')
    )

    use_action_server = LaunchConfiguration('use_action_server')
    use_action_server_arg = DeclareLaunchArgument(
        'use_action_server',
        default_value='false',
        description='Start managed behaviors through ROS action servers'
    )

    recover_thrusters_use_action_server = LaunchConfiguration(
        'recover_thrusters_use_action_server')
    recover_thrusters_use_action_server_arg = DeclareLaunchArgument(
        'recover_thrusters_use_action_server',
        default_value=use_action_server,
        description='Override use_action_server for the recover '
                    'thrusters node only'
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

    water_visibility_node = Node(
        package='suave_monitor',
        executable='water_visibility_observer',
        name='water_visibility_observer_node',
        parameters=[mission_config],
        output=print_output,
    )

    battery_monitor_node = Node(
        package='suave_monitor',
        executable='battery_monitor',
        name='battery_monitor',
        parameters=[mission_config],
        output=print_output,
    )

    thruster_monitor_node = Node(
        package='suave_monitor',
        executable='thruster_monitor',
        name='thruster_monitor',
        parameters=[mission_config],
        output=print_output,
    )

    pipeline_detection_wv_node = Node(
        package='suave',
        executable='pipeline_detection_wv',
        output=print_output,
    )

    spiral_search_node = Node(
        package='suave',
        executable='spiral_search',
        parameters=[{'use_action_server': use_action_server}],
        output=print_output,
    )

    follow_pipeline_node = Node(
        package='suave',
        executable='follow_pipeline',
        parameters=[{'use_action_server': use_action_server}],
        output=print_output,
    )

    recharge_battery_node = Node(
        package='suave',
        executable='recharge_battery',
        parameters=[{'use_action_server': use_action_server}],
        output=print_output,
    )

    recover_thrusters_node = Node(
        package='suave',
        executable='recover_thrusters',
        parameters=[{
            'use_action_server': recover_thrusters_use_action_server,
        }],
        output=print_output,
    )

    task_bridge_node = Node(
        package='suave',
        executable='task_bridge_none',
        name='task_bridge',
        parameters=[mission_config],
        output=print_output,
        condition=LaunchConfigurationEquals('task_bridge', 'True')
    )

    pkg_suave_path = get_package_share_directory('suave')

    system_modes_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'system_modes.launch.py')

    system_modes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_modes_launch_path),
        condition=LaunchConfigurationEquals('system_modes', 'True')
    )

    return LaunchDescription([
        task_bridge_arg,
        system_modes_arg,
        use_action_server_arg,
        recover_thrusters_use_action_server_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        print_output_arg,
        mission_config_arg,
        water_visibility_node,
        battery_monitor_node,
        pipeline_detection_wv_node,
        thruster_monitor_node,
        spiral_search_node,
        follow_pipeline_node,
        recharge_battery_node,
        recover_thrusters_node,
        task_bridge_node,
        system_modes_launch,
    ])
