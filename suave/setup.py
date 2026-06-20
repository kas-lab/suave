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

"""Package the core SUAVE managed-system nodes and resources."""

from glob import glob

import os

from setuptools import setup

package_name = 'suave'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gus',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pipeline_detection = suave.pipeline_node:main',
            'pipeline_detection_wv = suave.pipeline_detection_wv:main',
            'recharge_battery = suave.recharge_battery_lc:main',
            'spiral_search = suave.spiral_search_lc:main',
            'follow_pipeline = suave.follow_pipeline_lc:main',
            'recover_thrusters = suave.recover_thrusters_lc:main',
            'task_bridge_none = suave.task_bridge_none:main',
        ],
    },
)
