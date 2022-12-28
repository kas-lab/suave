import os
from glob import glob
from setuptools import setup

package_name = 'pipeline_inspection'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gus',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_node = ' +
                'pipeline_inspection.pipeline_node:main',
            'follow_pipeline = ' +
                'pipeline_inspection.follow_pipeline:main',
            'follow_waypoints = ' +
                'pipeline_inspection.follow_waypoints:main',
            'mission = ' +
                'pipeline_inspection.mission:mission',
            'spiral_mission = ' +
                'pipeline_inspection.spiral_mission:main',
            'spiral_pattern_position = ' +
                'pipeline_inspection.spiral_pattern_position:main',
            'thruster_failures = ' +
                'pipeline_inspection.thruster_failures:main',
        ],
    },
)
