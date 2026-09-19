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

"""Launch the generic batch runner from its YAML campaign manifest."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Return the configured batch-runner launch description."""
    # Get the path to the config file
    config_path = os.path.join(
        get_package_share_directory('suave_runner'),
        'config',
        'runner',
        'batch_campaigns.yml'
    )

    # Launch the suave_runner batch node with the campaigns loaded from YAML
    return LaunchDescription([
        Node(
            package='suave_runner',
            executable='run_batch',
            name='run_batch_node',
            output='screen',
            parameters=[config_path],
        )
    ])
