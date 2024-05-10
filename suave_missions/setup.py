import os
from glob import glob
from setuptools import setup

package_name = 'suave_missions'

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
    maintainer='ega',
    maintainer_email='e.g.alberts@',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'const_dist_mission = ' +
                'suave_missions.const_dist_mission:main',
            'time_constrained_mission = ' +
                'suave_missions.time_constrained_mission:main',
        ],
    },
)
