import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur5_isaac_simulation"), "urdf", "ur.urdf.xacro"]),
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=ur5",
            " ",
            "tf_prefix:=",
            "",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur5_isaac_simulation"), "config", "ur5_rviz.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output={'both': 'log'},
        arguments=["-d", rviz_config_file],
    )

    ur5_traj_server = Node(
        package='ur5_isaac_simulation',
        name='ur5_controller',
        executable='ur5_controller'
    )

    gripper_traj_server = Node(
        package='ur5_isaac_simulation',
        name='gripper_controller',
        executable='gripper_controller'
    )

    ur5_main_node = Node(
        package='ur5_isaac_simulation',
        name='ur5_isaac_ros2',
        executable='ur5_isaac_ros2'
    )

    nodes_to_start = [
        ur5_traj_server,
        gripper_traj_server,
        # ur5_main_node,
        robot_state_publisher_node,
        rviz_node,
    ]
    return LaunchDescription(nodes_to_start)
