import os
import time
import rclpy
import yaml
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


def load_yaml_file(filename) -> dict:
    """Load yaml file with the soft robot parameters"""
    with open(filename, 'r', encoding='UTF-8') as file:
        data = yaml.safe_load(file)
    return data


class TimeIt:
    """Class to measure the time of a block of code"""

    def __init__(self, message):
        self.message = message
        self.time_0 = None
        self.time_1 = None
        self.print_output = False

    def __enter__(self):
        self.time_0 = time.time()

    def __exit__(self, current_time, value, traceback):
        self.time_1 = time.time()
        print(f'{self.message}: {(self.time_1 - self.time_0)*1000:.5f} ms')


class UR5Isaac(Node):
    """Class to simulate UR5 Robot in Isaac Sim"""

    def __init__(self):
        """Initialize the UR5 Isaac Sim simulation"""

        self.node_name = "ur5_isaac_ros2"
        super().__init__(self.node_name)
        self.publisher_ = self.create_publisher(JointState, "ur5_joint_command", 10)
        self.joint_state = JointState()

        config_file = os.path.join(
            get_package_share_directory("ur5_isaac_simulation"),
            'config',
            'params.yaml'
        )
        config = load_yaml_file(config_file)
        self.ros_parameters = config[self.node_name]["ros__parameters"]

        declared_parameters = []
        for key, value in self.ros_parameters.items():
            declared_parameters.append((key, value))

        self.get_logger().info(f"{declared_parameters}")
        self.declare_parameters(namespace='',
                                parameters=declared_parameters)
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.simulate)

    def parameters_callback(self, params):
        # do some actions, validate parameters, update class attributes, etc.
        for param in params:
            if param.name == "home_joint_array":
                if param.type_ in [Parameter.Type.DOUBLE_ARRAY]:
                    self.ros_parameters['home_joint_array'] = list(param.value)
                else:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def simulate(self) -> None:
        """UR5 Isaac Sim Simulation"""
        self.get_logger().info(str(self.ros_parameters['home_joint_array']))


def main(args=None):
    """Main function to run the UR5 Isaac Sim simulation"""
    rclpy.init(args=args)

    ros2_publisher = UR5Isaac()
    rclpy.spin(ros2_publisher)

    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
