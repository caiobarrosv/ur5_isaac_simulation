import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from interactive_markers import InteractiveMarkerServer
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_matrix
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5_isaac_simulation.helper_functions import transformations
from ur5_isaac_simulation.helper_functions.interactive_marker import \
    InteractiveMarkerUR5
from ur5_isaac_simulation.helper_functions.inv_kin import inverse_kinematics
from ur5_isaac_simulation.helper_functions.load_ros_parameters import \
    get_ros_parameters

from .tkinter_gui import TkinterGui


class UR5Isaac(Node):
    """Class to simulate UR5 Robot in Isaac Sim"""

    ur5_joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    gripper_joint_names = [
        "right_outer_knuckle_joint",
        "left_outer_knuckle_joint"
    ]

    def __init__(self):
        """Initialize the UR5 Isaac Sim simulation"""

        node_name = "ur5_isaac_ros2"
        super().__init__(node_name)

        ###############################
        # TF LISTENER
        ###############################
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        ###############################
        # UR5 ACTION CLIENT
        ###############################
        self._action_client = ActionClient(self,
                                           FollowJointTrajectory,
                                           'ur5/follow_joint_trajectory')

        ###############################
        # GRIPPER ACTION CLIENT
        ###############################
        self._gripper_action_client =\
            ActionClient(self,
                         FollowJointTrajectory,
                         'ur5_gripper/follow_joint_trajectory')

        ###############################
        # ROS PARAMETERS
        ###############################
        self.ros_parameters, declared_parameters =\
            get_ros_parameters(node_name)
        self.declare_parameters(namespace='',
                                parameters=declared_parameters)
        self.get_logger().info(f"{declared_parameters}")
        self.add_on_set_parameters_callback(self.parameters_callback)

        ###############################
        # MARKER
        ###############################
        self.interactive_marker_server =\
            InteractiveMarkerServer(node=self,
                                    namespace='interactive_markers')
        self.iteractive_marker_ur5 =\
            InteractiveMarkerUR5(self.interactive_marker_server,
                                 self.send_goal,
                                 self.get_transform_between_frames,
                                 self.ros_parameters)
        self.iteractive_marker_ur5.add_interactive_maker("base_link_inertia",
                                                         "base_link_inertia")

        ###############################
        # TKINTER GUI
        ###############################
        self.tkinter = TkinterGui(self.send_goal, self.send_goal_gripper)

        self.inv_kin = False
        self.debug_inv_kin = False
        self.goal_prim = ''

        self.root = self.tkinter.build_frames()
        timer_period = 1/60
        self.timer = self.create_timer(timer_period, self.simulate)

    def send_goal(
        self,
        target: list,
        movement: str = "slow",
        inv_kin: bool = False,
        solution_index: int = 5,
        debug_inv_kin: bool = False
    ):
        """
        Send trajectory to the UR5 robot in Isaac Sim.

        Parameters
        ----------
        target : list
            Target joint positions or
            target pose [x, y, z, roll, pitch, yaw] in degrees (if inv_kin
            is True).
        movement : str
            Type of movement (slow, fast).
        inv_kin : bool
            If True, the target is the desired pose
            (default: {False}).
        solution_index: int
            Index of the best solution for the Laboratory of Robotics at UFBA.
        debug_inv_kin: bool
            If True, print the joint angles acquired through the inverse
            kinematics.

        """
        self.goal_prim = 'UR5'
        if movement == 'slow':
            time_in_sec = self.ros_parameters['trajectory_time_slow']
        elif movement == 'fast':
            time_in_sec = self.ros_parameters['trajectory_time_fast']

        if inv_kin:
            self.get_logger().info(
                f"[UR5] Target position [x, y, z]: {target[:3]}")
            self.get_logger().info(
                f"[UR5] Target orientation [roll, pitch, yaw]: {target[3:]}")
            desired_pose =\
                transformations.get_desired_pose_htm(
                    position=np.array(target[:3]),
                    roll=target[3],
                    pitch=target[4],
                    yaw=target[5]
                )
            target = inverse_kinematics(desired_pose,
                                        print_debug=debug_inv_kin,
                                        solution_index=solution_index)

        if time_in_sec is not None:
            joint_names = UR5Isaac.ur5_joint_names
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = joint_names
            duration = Duration(sec=time_in_sec)
            goal.trajectory.points.append(
                JointTrajectoryPoint(positions=target,
                                     velocities=[0.0]*6,
                                     accelerations=[0.0]*6,
                                     time_from_start=duration))
            self._action_client.wait_for_server(timeout_sec=10.0)
            send_goal_future =\
                self._action_client.send_goal_async(
                    goal,
                    feedback_callback=self.feedback_callback)
            send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(
                f"[{self.goal_prim}] Joint goal [{target}] sent to the UR5 robot.")
            self.inv_kin = inv_kin
            self.debug_inv_kin = debug_inv_kin
        else:
            self.get_logger().error(
                "Please, specify the time in seconds or"
                "the type of movement (slow or fast)")

    def send_goal_gripper(
        self,
        position: list
    ):
        """
        Send trajectory to the UR5 gripper in Isaac Sim.

        Parameters
        ----------
        time_in_sec : float
            Time in seconds to complete the action.
        position : list
            List of joint positions.

        """
        self.goal_prim = 'GRIPPER'
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_names
        duration = Duration(sec=3)
        n_joints = len(self.gripper_joint_names)
        goal.trajectory.points.append(
            JointTrajectoryPoint(positions=position,
                                 velocities=[0.0]*n_joints,
                                 accelerations=[0.0]*n_joints,
                                 time_from_start=duration))
        self._gripper_action_client.wait_for_server(timeout_sec=10.0)
        send_goal_future =\
            self._gripper_action_client.send_goal_async(
                goal,
                feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            f"[{self.goal_prim}] Joint goal [{position}] sent to the UR5 robot.")

    def goal_response_callback(self, future):
        """
        Receives the response from the action server.

        Parameters
        ----------
        future : rclpy.task.Future
            Future object with the response from the action server

        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected :(')
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Receives the result from the action server.

        Parameters
        ----------
        future : rclpy.task.Future
            Future object with the result from the action server

        """
        result = future.result().result
        self.get_logger().info(
            f'[Feedback] [{self.goal_prim}] Result: {result.error_string}')
        self.debug_htm_wrist_3()

    def get_transform_between_frames(
        self,
        target_frame: str,
        source_frame: str,
        pose_to_transform: PoseStamped = None
    ):
        """
        Get the transform between two frames.

        Parameters
        ----------
        target_frame : str
            Target frame name. Example: "base_link_inertia"
        source_frame : str
            Source frame name. Example: "wrist_3_link"
        pose_to_transform : PoseStamped
            Pose to be transformed to the target frame

        Returns
        -------
        geometry_msgs.msg.TransformStamped
            Transform between the two frames
        geometry_msgs.msg.TransformStamped or None
            Returns pose transformed to the target frame if pose_to_transform
            is not None

        """
        try:
            pose = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            pose_transformed = None
            if pose_to_transform is not None:
                pose_transformed = do_transform_pose(pose_to_transform, pose)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
            return
        return pose, pose_transformed

    def debug_htm_wrist_3(self):
        """Debug the inv. kin. using the direct kinematics as input."""
        if self.debug_inv_kin:
            # If True, print the joint angles acquired through the inverse
            # kinematics using the direct kinematics as input
            # (from Tkinter). It is useful to check if the inverse kinematics
            # is working properly.
            geom_msg_to_htm = transformations.geometry_msg_pose_to_htm
            position_link_06, _ =\
                self.get_transform_between_frames("base_link_inertia",
                                                  "wrist_3_link")
            if position_link_06 is not None:
                htm_link_06 = geom_msg_to_htm(position_link_06.transform)
                euler_angles = euler_from_matrix(htm_link_06, 'sxyz')
                self.get_logger().info(
                    f"Received Euler angles: {np.degrees(euler_angles)}")
                self.get_logger().info(f"htm_link_06:\n{htm_link_06}")
                inverse_kinematics(htm_link_06, print_debug=True)

    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server.

        Parameters
        ----------
        feedback_msg : FollowJointTrajectory.Feedback
            Feedback message from the action server

        """
        feedback = feedback_msg.feedback
        info = self.get_logger().info
        des_positions = np.round(feedback.desired.positions, 5)
        actual_positions = np.round(feedback.actual.positions, 5)
        error_positions = np.round(feedback.error.positions, 5)
        info(
            f'[Feedback] [{self.goal_prim}] Desired position: {des_positions}')
        info(
            f'[Feedback] [{self.goal_prim}] Reached position: {actual_positions}')
        info(
            f'[Feedback] [{self.goal_prim}] Error position by joint: {error_positions}')
        info(f"[Feedback] [{self.goal_prim}] Absolute error (radians): "
             f"{abs(sum(feedback.error.positions))}")

    def parameters_callback(self, params):
        """
        Update ROS2 parameters according to the config/params.yaml file.

        Parameters
        ----------
        params : list
            List of parameters to be updated

        Returns
        -------
        SetParametersResult
            Result of the update of the parameters

        """
        for param in params:
            if param.name == "home_joint_array":
                if param.type_ in [Parameter.Type.DOUBLE_ARRAY]:
                    self.ros_parameters['home_joint_array'] = list(param.value)
                else:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def simulate(self) -> None:
        """UR5 Isaac Sim Simulation."""
        self.root.update()


def main(args=None):
    """Main function to run the UR5 Isaac Sim simulation."""
    rclpy.init(args=args)

    ros2_publisher = UR5Isaac()
    rclpy.spin(ros2_publisher)

    ros2_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
