from setuptools import setup
import os

PACKAGE_NAME = 'ur5_isaac_simulation'
PACKAGES_LIST = [
    PACKAGE_NAME,
    'ur5_isaac_simulation.helper_functions',
    'ur5_isaac_simulation.action_servers',
]

DATA_FILES = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml'])
    ]


def package_files(data_files, directory_list):
    """
    Get all files in a directory and subdirectory and return a list of tuples.

    Parameters
    ----------
    data_files : list
        List of tuples containing the path to install the files and the files
        themselves.
    directory_list : list
        List of directories to get the files from.

    Returns
    -------
    data_files : list
        List of tuples containing the path to install the files and the files
        themselves.

    """
    paths_dict = {}
    for directory in directory_list:
        for (path, _, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', PACKAGE_NAME, path)

                if install_path in paths_dict:
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key, value in paths_dict.items():
        data_files.append((key, value))

    return data_files


setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=PACKAGES_LIST,
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
            # Action Servers
            'ur5_controller = ur5_isaac_simulation.action_servers.ur5_controller:main',
            'gripper_controller = ur5_isaac_simulation.action_servers.gripper_controller:main',

            # Nodes
            'ur5_isaac_ros2 = ur5_isaac_simulation.ur5_isaac_ros2:main'
        ],
    },
)
