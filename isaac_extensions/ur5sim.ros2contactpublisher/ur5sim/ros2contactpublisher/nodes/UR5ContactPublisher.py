"""
This is the implementation of the OGN node defined in UR5ContactPublisher.ogn
"""
import rclpy
from std_msgs.msg import Float32MultiArray

rclpy.init()
node = rclpy.create_node('ur5_contact_node')
pub = node.create_publisher(Float32MultiArray, 'ur5_contact_publisher', 10)

class UR5ContactPublisher:
    """Publish contacts of the robotic gripper 2F-140."""

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
