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

"""Declare common SUAVE mission bringup arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Return common mission-selection launch arguments."""
    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge, none/metacontrol/random')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
    ])
