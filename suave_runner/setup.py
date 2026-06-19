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

"""Package SUAVE experiment orchestration and analysis tools."""

import os
from glob import glob
from setuptools import setup

package_name = 'suave_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                (os.path.join('share', package_name), ['package.xml']),
                (os.path.join('share', package_name,
                              'launch'), glob('launch/*launch.[pxy][yma]*')),
                (os.path.join('share', package_name,
                              'config'), glob('config/*')),
                (os.path.join('share', package_name, 'test'),
                 glob('test/*.py')), ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts':
        ['suave_runner = suave_runner.suave_runner:main',
         'statistical_analysis = suave_runner.statistical_analysis:main'],
    },
)
