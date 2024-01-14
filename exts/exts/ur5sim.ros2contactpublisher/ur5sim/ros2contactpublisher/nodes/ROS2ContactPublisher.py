"""
This is the implementation of the OGN node defined in ROS2ContactPublisher.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import rclpy
from std_msgs.msg import Float32MultiArray
import carb

rclpy.init()
node = rclpy.create_node('ur5_contact_node')
pub = node.create_publisher(Float32MultiArray, 'ur5_contact_publisher', 10)
carb.log_warn("rclpy loaded and node ur5_contact_node created")


class ROS2ContactPublisher:
    """
         Publishes contact and force values
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            data = [
                db.inputs.left_force_value,
                db.inputs.right_force_value,
                db.inputs.left_contact_bool,
                db.inputs.right_contact_bool
            ]
            msg_to_pub = Float32MultiArray(data=data)
            pub.publish(msg_to_pub)
        except Exception as error:
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs
        # mean success
        return True
