import os
from glob import glob
from setuptools import setup

package_name = 'suave_metacontrol'

setup(
    name=package_name,
    version='0.23.0',
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
    maintainer='jeroen',
    maintainer_email='j.zwanepol@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spiral_lc_node = ' +
                'suave_metacontrol.spiral_lc_node:main',
            'fake_managed_system = ' +
                'suave_metacontrol.fake_managed_system:main',
            'water_visibility_observer = ' +
                'suave_metacontrol.' +
                'water_visibility_observer:main',
            'pipeline_detection_wv = ' +
                'suave_metacontrol.' +
                'pipeline_detection_wv:main',
            'follow_pipeline_lc = ' +
                'suave_metacontrol.follow_pipeline_lc:main',
            'const_dist_mission = ' +
                'suave_metacontrol.const_dist_mission:main',
            'const_dist_mission_no_adapt = ' +
                'suave_metacontrol.const_dist_mission_no_adapt:main',
            'time_constrained_mission = ' +
                'suave_metacontrol.time_constrained_mission:main',
            'time_constrained_mission_no_adapt = ' +
                'suave_metacontrol.time_constrained_mission_no_adapt:main',
            'suave_reasoner = ' +
                'suave_metacontrol.' +
                'suave_reasoner:main',
        ],
    },
)
