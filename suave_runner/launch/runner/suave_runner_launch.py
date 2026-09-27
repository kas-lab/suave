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

"""Launch the SUAVE experiment runner from its YAML configuration."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the configured experiment-runner launch description."""
    config_file = LaunchConfiguration('config_file')

    # Get the path to the config file
    config_path = os.path.join(
        get_package_share_directory('suave_runner'),
        'config',
        'runner',
        'runner_config.yml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_path,
        description='Full path to the suave_runner YAML configuration file')

    # Launch the suave_runner node with the parameters loaded from YAML
    return LaunchDescription([
        config_file_arg,
        Node(
            package='suave_runner',
            executable='suave_runner',
            name='suave_runner_node',
            output='screen',
            parameters=[config_file],
        )
    ])
