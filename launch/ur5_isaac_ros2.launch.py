import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('ur5_isaac_simulation'),
        'config',
        'params.yaml'
        )
    
    print(config)
    node=Node(
        package = 'ur5_isaac_simulation',
        name = 'ur5_isaac_ros2',
        executable = 'ur5_isaac_ros2',
        parameters = [config]
    )
    ld.add_action(node)
    return ld