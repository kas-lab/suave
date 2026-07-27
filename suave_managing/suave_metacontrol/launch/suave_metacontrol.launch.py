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

"""Launch the metacontrol adaptation manager."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the metacontrol manager launch description."""
    tomasys_file = LaunchConfiguration('tomasys_file')
    model_file = LaunchConfiguration('model_file')
    reasoning_time_filename = LaunchConfiguration('reasoning_time_filename')
    result_path = LaunchConfiguration('result_path')

    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')
    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    tomasys_files = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl')]
    suave_ontology_path = os.path.join(
        pkg_suave_metacontrol_path, 'config', 'suave.owl')
    metacontrol_config = os.path.join(
        pkg_suave_metacontrol_path, 'config', 'metacontrol_config.yaml')

    tomasys_file_arg = DeclareLaunchArgument(
        'tomasys_file',
        default_value=str(tomasys_files),
        description='Paths for the TOMASys ontologies'
    )
    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value=suave_ontology_path,
        description='Path for the application-specific ontology file'
    )
    reasoning_time_filename_arg = DeclareLaunchArgument(
        'reasoning_time_filename',
        default_value='metacontrol_reasoning_time',
        description='File name for saving metacontrol reasoning time'
    )
    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path where to save the reasoning time'
    )

    mros_reasoner_node = Node(
        package='mros2_reasoner',
        executable='mros2_reasoner_node',
        name='suave_reasoner',
        output='screen',
        parameters=[metacontrol_config, {
            'tomasys_file': tomasys_file,
            'model_file': model_file,
            'reasoning_time_filename': reasoning_time_filename,
            'reasoning_time_file_path': result_path,
        }],
    )
    mros_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )
    task_bridge_node = Node(
        package='suave_metacontrol',
        executable='task_bridge_metacontrol',
    )

    return LaunchDescription([
        tomasys_file_arg,
        model_file_arg,
        reasoning_time_filename_arg,
        result_path_arg,
        mros_reasoner_node,
        mros_system_modes_bridge_node,
        task_bridge_node,
    ])
