import os
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml_file(filename) -> dict:
    """Load yaml file with the UR5 Isaac Sim Parameters"""
    with open(filename, 'r', encoding='UTF-8') as file:
        data = yaml.safe_load(file)
    return data


def get_ros_parameters(node_name):
    """Get the ROS2 parameters from the yaml file

    Returns
    -------
    dict
        ROS2 parameters
    list
        Declared parameters

    """
    # Get the parameters from the yaml file
    config_file = os.path.join(
        get_package_share_directory("ur5_isaac_simulation"),
        'config',
        'params.yaml'
    )
    config = load_yaml_file(config_file)
    ros_parameters = config[node_name]["ros__parameters"]

    # Declare the parameters in the ROS2 parameter server
    declared_parameters = []
    for key, value in ros_parameters.items():
        declared_parameters.append((key, value))
    return ros_parameters, declared_parameters
