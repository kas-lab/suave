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
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_detection = suave.pipeline_node:main',
            'pipeline_detection_wv = suave.pipeline_detection_wv:main',
            'spiral_search = suave.spiral_search_lc:main',
            'follow_pipeline = suave.follow_pipeline_lc:main',
            'thruster_monitor = suave.thruster_monitor:main',
            'recover_thrusters = suave.recover_thrusters_lc:main',
            'water_visibility_observer = suave.water_visibility_observer:main',
            'task_bridge_none = suave.task_bridge_none:main',
        ],
    },
)
