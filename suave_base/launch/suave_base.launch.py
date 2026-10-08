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

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# Topics published or subscribed by SUAVE nodes, MAVROS, the Gazebo bridges,
# and the system_modes mode manager. Every node runs in the root namespace.
RECORDED_TOPICS = [
    '/diagnostics',
    '/parameter_events',
    # Mission progress
    '/pipeline/detected',
    '/pipeline/inspected',
    '/pipeline/distance_inspected',
    '/battery_monitor/recharge/complete',
    # MAVROS
    '/mavros/state',
    '/mavros/local_position/pose',
    '/mavros/setpoint_position/local',
    # Gazebo bridges (/ocean_current only when enable_water_current is true)
    '/model/bluerov2/pose',
    '/model/min_pipes_pipeline/pose',
    '/ocean_current',
    # Metrics
    '/mission_metrics/done',
    '/mission_metrics/reaction_time',
    # Requirement monitor
    '/requirements/thruster_availability/fulfillment',
    '/requirements/search_footprint/fulfillment',
    '/requirements/adaptation_reaction_time/fulfillment',
    '/requirements/adaptation_reaction_time/thruster/fulfillment',
    '/requirements/adaptation_reaction_time/water_visibility/fulfillment',
    '/requirements/adaptation_reaction_time/battery/fulfillment',
    # Lifecycle transitions of the managed functions
    '/f_generate_search_path_node/transition_event',
    '/f_follow_pipeline_node/transition_event',
    '/f_maintain_motion_node/transition_event',
    '/generate_recharge_path_node/transition_event',
    # system_modes mode changes
    '/f_generate_search_path/mode_event',
    '/f_follow_pipeline/mode_event',
    '/f_maintain_motion/mode_event',
    '/generate_recharge_path/mode_event',
]


def record_bags(context, *args, **kwargs):
    """Return a recorder writing to <result_path>/rosbags/<run_name>."""
    result_path = os.path.expanduser(
        LaunchConfiguration('result_path').perform(context))
    prefix = LaunchConfiguration('result_filename').perform(context)
    run_name = '{}_{}'.format(
        prefix or 'suave', datetime.now().strftime('%Y%m%d_%H%M%S'))
    rosbags_path = os.path.join(result_path, 'rosbags')
    # ros2 bag creates the run folder and fails if it already exists.
    os.makedirs(rosbags_path, exist_ok=True)
    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-s', 'mcap',
            '--storage-preset-profile', 'fastwrite',
            '-o', os.path.join(rosbags_path, run_name),
            *RECORDED_TOPICS,
        ],
        name='rosbag_record',
        # Give the recorder time to finalize the bag after SIGINT.
        sigterm_timeout='20',
    )]


def generate_launch_description():
    """Return the base SUAVE launch description."""
    mission_config = LaunchConfiguration('mission_config')
    use_action_server = LaunchConfiguration('use_action_server')
    enable_water_current = LaunchConfiguration('enable_water_current')
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
            'enable_water_current',
            default_value='false',
            description='Enable the sinusoidal ocean-current publisher'),
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
        DeclareLaunchArgument(
            'record_bags',
            default_value='true',
            description='Record SUAVE topics to an MCAP bag under '
                        '<result_path>/rosbags'),
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
            'enable_water_current': enable_water_current,
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

    requirement_monitor_node = Node(
        package='suave_requirements',
        executable='requirement_monitor',
        name='requirement_monitor')

    bag_recorder = OpaqueFunction(
        function=record_bags,
        condition=IfCondition(LaunchConfiguration('record_bags')))

    return LaunchDescription([
        *arguments,
        managed_system,
        mission_metrics_node,
        mission_metrics_node_filename,
        requirement_monitor_node,
        bag_recorder,
    ])
