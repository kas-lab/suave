from setuptools import find_packages, setup

package_name = 'suave_metrics'

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
    description='Package for collecting metrics of SUAVE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_metrics = suave_metrics.mission_metrics:main'
        ],
    },
)
