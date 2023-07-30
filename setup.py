from setuptools import setup
from glob import glob
import os

PACKAGE_NAME = 'ur5_isaac_simulation'
HELPER_FUNC_SUB_MODULE = 'ur5_isaac_simulation/helper_functions'

DATA_FILES = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml'])
    ]

def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', PACKAGE_NAME, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME, HELPER_FUNC_SUB_MODULE],
    data_files=package_files(DATA_FILES, ['meshes/', 'config/', 'urdf/',
                                          'launch/', 'srv/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Caio Viturino',
    maintainer_email='engcaiobarros@gmail.com',
    description='UR5 Isaac Sim ROS2 Humble simulation Package',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5_isaac_ros2 = ur5_isaac_simulation.ur5_isaac_ros2:main',
            'ur5_controller = ur5_isaac_simulation.ur5_controller:main',
            'interactive_marker = ur5_isaac_simulation.interactive_marker:main'
        ],
    },
)
