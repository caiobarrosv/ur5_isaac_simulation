"""
OmniGraph Script Node to publish contact sensor data directly to ROS2.

Inputs:
    left_force (float): Force on left gripper pad
    right_force (float): Force on right gripper pad
    left_contact (float): Left pad contact (1.0 or 0.0)
    right_contact (float): Right pad contact (1.0 or 0.0)

Outputs:
    None - publishes directly to ROS2 topic /ur5_contact_publisher
"""


class InternalState:
    """Maintains state between compute calls"""
    def __init__(self):
        self.publisher = None
        self.node = None
        self.topic_name = "/ur5_contact_publisher"


def setup(db):
    """Initialize the ROS2 publisher"""
    state = db.per_instance_state
    state.publisher = None
    state.node = None
    state.topic_name = "/ur5_contact_publisher"
    return True


def compute(db):
    """Publish contact sensor data to ROS2"""
    try:
        state = db.per_instance_state

        # Get all input values
        left_force = float(db.inputs.left_force)
        right_force = float(db.inputs.right_force)
        left_contact = float(db.inputs.left_contact)
        right_contact = float(db.inputs.right_contact)

        # Create data array in correct order
        contact_data = [left_force, right_force, left_contact, right_contact]

        # Lazy initialization of ROS2 publisher
        if state.publisher is None:
            try:
                # Import ROS2 modules
                import rclpy
                from std_msgs.msg import Float32MultiArray
                from rclpy.qos import QoSProfile

                # Initialize rclpy if not already done
                if not rclpy.ok():
                    rclpy.init()

                # Create a node for this publisher
                state.node = rclpy.create_node('contact_sensor_publisher_script')

                # Create publisher with QoS profile
                qos_profile = QoSProfile(depth=10)
                state.publisher = state.node.create_publisher(
                    Float32MultiArray,
                    state.topic_name,
                    qos_profile
                )

                db.log_info(f"ROS2 publisher created for topic: {state.topic_name}")

            except Exception as e:
                db.log_error(f"Failed to create ROS2 publisher: {str(e)}")
                return False

        # Publish the message
        if state.publisher:
            from std_msgs.msg import Float32MultiArray
            msg = Float32MultiArray()
            msg.data = contact_data
            state.publisher.publish(msg)

        return True

    except Exception as e:
        db.log_error(f"Contact publisher script error: {str(e)}")
        return False


def cleanup(db):
    """Cleanup ROS2 resources"""
    try:
        state = db.per_instance_state
        if state.publisher:
            state.publisher.destroy()
            state.publisher = None
        if state.node:
            state.node.destroy_node()
            state.node = None
    except Exception as e:
        pass
    return True
