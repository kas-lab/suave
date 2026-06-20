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

"""Launch the extended Behavior Tree adaptation manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the extended Behavior Tree manager launch description."""
    mission_config = LaunchConfiguration('mission_config')
    use_action_server = LaunchConfiguration('use_action_server')

    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        description='Mission configuration file'
    )
    use_action_server_arg = DeclareLaunchArgument(
        'use_action_server',
        default_value='false',
        description='Start managed behaviors through ROS action servers'
    )

    suave_bt_node = Node(
        package='suave_bt',
        executable='suave_bt_extended',
        parameters=[mission_config, {
            'use_action_server': use_action_server,
        }]
    )

    return LaunchDescription([
        mission_config_arg,
        use_action_server_arg,
        suave_bt_node,
    ])
