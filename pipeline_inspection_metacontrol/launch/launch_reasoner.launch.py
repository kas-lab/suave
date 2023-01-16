# Copyright (c) 2018 Intel Corporation
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    # Create the launch configuration variables
    tomasys_file = LaunchConfiguration('tomasys_file')
    model_file = LaunchConfiguration('model_file')
    desired_configuration = LaunchConfiguration('desired_configuration')

    tomasys_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl')]

    tomasys_file_arg = DeclareLaunchArgument(
        'tomasys_file',
        default_value=str(tomasys_files_array),
        description='tomasys ontologies'
    )

    mock_ontology_path = os.path.join(
        pkg_mros_ontology_path, 'owl', 'mock.owl')

    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value=mock_ontology_path,
        description='File name for the Working ontology file')

    desired_configuration_arg = DeclareLaunchArgument(
        'desired_configuration',
        default_value='',
        description='Desired inital configuration (system mode)')

    mros_reasoner_node = Node(
        package='pipeline_inspection_metacontrol',
        executable='pipeline_inspection_reasoner',
        name='pipeline_inspection_reasoner',
        output='screen',
        parameters=[{
            'tomasys_file': tomasys_file,
            'model_file': model_file,
            'desired_configuration': desired_configuration,
        }],
    )

    return LaunchDescription([
        tomasys_file_arg,
        model_file_arg,
        desired_configuration_arg,
        mros_reasoner_node
    ])
