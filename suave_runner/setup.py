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
                              'config'), glob('config/*'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['suave_runner = suave_runner.suave_runner:main'],
    },
)
