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

"""Launch the SUAVE system-modes manager."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Return the system-modes manager launch description."""
    shm_model_path = os.path.join(
        get_package_share_directory('suave'),
        'config',
        'suave_modes.yaml')

    mode_manager_node = Node(
        package='system_modes',
        executable='mode_manager',
        parameters=[{'modelfile': shm_model_path}],
    )

    return LaunchDescription([
        mode_manager_node,
    ])
