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

"""Select and launch a SUAVE mission with an adaptation manager."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return the top-level configurable mission launch description."""
    mc_reasoning_time_filename = LaunchConfiguration(
        'mc_reasoning_time_filename')

    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge' +
                    '[none, metacontro, random, or bt]'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Desired mission type'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    battery_constraint_arg = DeclareLaunchArgument(
        'battery_constraint',
        default_value='False',
        description='Desired battery functionality' +
                    '[True or False]'
    )

    battery_constraint_value_arg = DeclareLaunchArgument(
        'battery_constraint_value',
        default_value='0.2',
        description='battery constraint value'
    )

    mc_reasoning_time_filename_arg = DeclareLaunchArgument(
        'mc_reasoning_time_filename',
        default_value='metacontrol_reasoning_time',
        description='metacontrol reasoning time filename'
    )

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')
    suave_metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_metacontrol.launch.py'
    )

    pkg_suave_random_path = get_package_share_directory(
        'suave_random')
    suave_random_launch_path = os.path.join(
        pkg_suave_random_path,
        'launch',
        'suave_random.launch.py'
    )

    pkg_suave_none_path = get_package_share_directory(
        'suave_none')
    suave_none_launch_path = os.path.join(
        pkg_suave_none_path,
        'launch',
        'suave_none.launch.py'
    )

    pkg_suave_bt_path = get_package_share_directory(
        'suave_bt')
    suave_bt_launch_path = os.path.join(
        pkg_suave_bt_path,
        'launch',
        'suave_bt.launch.py'
    )

    suave_metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_metacontrol_launch_path),
        launch_arguments={
            'reasoning_time_filename': mc_reasoning_time_filename}.items(),
        condition=LaunchConfigurationEquals(
            'adaptation_manager', 'metacontrol'))

    suave_random_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_random_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'random'))

    suave_none_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_none_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'none'))

    suave_bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_bt_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'bt'))

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
        result_filename_arg,
        battery_constraint_arg,
        battery_constraint_value_arg,
        mc_reasoning_time_filename_arg,
        suave_metacontrol_launch,
        suave_random_launch,
        suave_none_launch,
        suave_bt_launch,
    ])
