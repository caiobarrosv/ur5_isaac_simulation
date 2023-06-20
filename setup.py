from setuptools import setup
from glob import glob
import os

package_name = 'ur5_isaac_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Caio Viturino',
    maintainer_email='engcaiobarros@gmail.com',
    description='UR5 Isaac Sim ROS2 Humble simulation Package',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_isaac_ros2 = ur5_isaac_simulation.ur5_isaac_ros2:main'
        ],
    },
)
