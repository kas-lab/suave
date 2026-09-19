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

"""Launch paired Wilcoxon analysis for a sorted extended_exp1 campaign."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """Configure the paired analysis node from a sorted extended_exp1 campaign."""
    share = Path(get_package_share_directory('suave_runner'))
    config = (share / 'config' / 'analysis' /
              'extended_exp1_analysis_config.yml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'results_root',
            description='Directory with run_idx-sorted CSVs for '
                        'extended_exp1, e.g. the sort_results.py --output '
                        'directory or a batch campaigns/extended_exp1/sorted/ '
                        'directory'),
        DeclareLaunchArgument(
            'output_root', default_value=LaunchConfiguration('results_root'),
            description='Directory for output CSVs'),
        DeclareLaunchArgument(
            'correction', default_value='holm', choices=['holm', 'none'],
            description='Correction across both metrics and all method pairs'),
        Node(
            package='suave_runner', executable='wilcoxon_analysis',
            name='wilcoxon_analysis', output='screen',
            parameters=[ParameterFile(str(config), allow_substs=True)]),
    ])
