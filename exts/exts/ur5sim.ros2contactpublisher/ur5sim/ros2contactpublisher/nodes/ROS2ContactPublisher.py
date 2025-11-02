"""
This is the implementation of the OGN node defined in ROS2ContactPublisher.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import rclpy
from std_msgs.msg import Float32MultiArray
import carb

# Module-level variables - initialized lazily on first use
_node = None
_publisher = None
_rclpy_initialized = False


class ROS2ContactPublisher:
    """
         Publishes contact and force values
    """
    @staticmethod
    def internal_state():
        """Returns an internal state dictionary for this node"""
        return {}

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        global _node, _publisher, _rclpy_initialized

        try:
            # Lazy initialization: create node and publisher on first use
            if _node is None:
                if not _rclpy_initialized:
                    try:
                        rclpy.init()
                        _rclpy_initialized = True
                    except RuntimeError:
                        # rclpy already initialized (e.g., by isaacsim.ros2.bridge)
                        pass

                _node = rclpy.create_node('ur5_contact_node')
                _publisher = _node.create_publisher(Float32MultiArray, 'ur5_contact_publisher', 10)
                carb.log_warn("ROS2 node 'ur5_contact_node' created and publisher initialized")

            # Publish the data
            data = [
                db.inputs.left_force_value,
                db.inputs.right_force_value,
                db.inputs.left_contact_bool,
                db.inputs.right_contact_bool
            ]
            msg_to_pub = Float32MultiArray(data=data)
            _publisher.publish(msg_to_pub)

        except Exception as error:
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs
        # mean success
        return True

    @staticmethod
    def release():
        """Clean up resources when node is removed"""
        global _node, _publisher, _rclpy_initialized

        if _node is not None:
            try:
                _node.destroy_node()
                carb.log_warn("ROS2 node 'ur5_contact_node' destroyed")
            except Exception as e:
                carb.log_error(f"Error destroying ROS2 node: {e}")
            finally:
                _node = None
                _publisher = None
