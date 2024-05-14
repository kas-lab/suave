from setuptools import find_packages, setup

package_name = 'suave_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gus',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Monitor nodes for SUAVE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_monitor = suave_monitor.thruster_monitor:main',
            'battery_monitor = suave_monitor.battery_monitor:main',
            'water_visibility_observer = suave_monitor.water_visibility_observer:main',
        ],
    },
)
