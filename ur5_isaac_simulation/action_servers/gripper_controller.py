#!/usr/bin/env python3
"""
Create and action server for the UR5 gripper. The action server is called
'ur5_gripper/follow_joint_trajectory' and it is used to control the gripper
joints in Isaac Sim.
"""
import copy
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from ur5_isaac_simulation.helper_functions import trajectory_check as tc
from ur5_isaac_simulation.helper_functions.load_ros_parameters import \
    get_ros_parameters


class UR5GripperTrajController(Node):
    """
    Create and handle the action 'follow_joint_trajectory' server.

    Joint relationships in IsaacSim (gripper left side):
    - robotiq_arg2f_base_link -> left_inner_knuckle_joint (Revolute)
    - robotiq_arg2f_base_link -> left_outer_knuckle_joint (Revolute)
    - left_outer_knuckle_joint -> left_outer_finger_joint (fixed)
    - left_outer_finger -> left_inner_finger (Revolute)
    - left_inner_finger -> left_inner_finger_pad_joint (fixed)

    """

    joint_names = ["finger_joint", "right_outer_knuckle_joint"]

    def __init__(self):
        super().__init__('gripper_controler_sever')

        ###############################
        # ROS PARAMETERS
        ###############################
        self.ros_parameters, declared_parameters =\
            get_ros_parameters("gripper_controler_sever")
        self.declare_parameters(namespace='',
                                parameters=declared_parameters)
        for param, value in declared_parameters:
            self.get_logger().info(f"\t[GRIPPER] {param}: {value}")
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.robot_isaac_pub = self.create_publisher(JointState,
                                                     "/joint_command",
                                                     10)

        client_cb_group = ReentrantCallbackGroup()
        self.joint_states = None
        self.create_subscription(JointState,
                                 '/joint_states',
                                 self.update_joint_state,
                                 10,
                                 callback_group=client_cb_group)

        self.contact_state = None
        self.create_subscription(Float32MultiArray,
                                 '/ur5_contact_publisher',
                                 self.update_contact_state,
                                 10,
                                 callback_group=client_cb_group)

        self.robotserver = ActionServer(self,
                                        FollowJointTrajectory,
                                        "ur5_gripper/follow_joint_trajectory",
                                        self.execute_callback,
                                        callback_group=client_cb_group)
        self.get_logger().info('gripper_controler_sever [ON]!')

        self.actual_joint_state = []
        self.trajectory_position_list = []
        self.trajectory_velocity_list = []
        self.trajectory_acceleration_list = []
        self.time_from_start_list = []
        self.time_t0 = 0.0
        self.timout = 0.0
        self.matrix_a = None
        self.actual_trajectory_to_send = JointState()
        self.received_goal_handle = None
        self.gripper_initially_in_contact = False
        self.succeed = False
        self.trajectory_in_execution = False
        self.n_joints = len(self.joint_names)
        self.joint_directions_close =\
            self.ros_parameters['joint_directions_close']

    def parameters_callback(
        self,
        params: list[Parameter]
    ) -> SetParametersResult:
        """Update ROS2 parameters according to the config/params.yaml file."""
        for param in params:
            if param.name == "goal_tolerance":
                if param.type_ in [Parameter.Type.DOUBLE_ARRAY]:
                    self.ros_parameters['goal_tolerance'] = list(param.value)
                else:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def update_joint_state(
        self,
        msg: JointState
    ):
        """
        Get simulated Robotiq gripper joint angles in the following order:

        shoulder_pan_joint, shoulder_lift_joint, elbow_joint
        wrist_1_joint, wrist_2_joint, wrist_3_joint

        Parameters
        ----------
        msg : JointState
            JointState message from the /joint_states topic

        """
        joint_positions = msg.position
        right_outer_knuckle_joing = joint_positions[-3]
        finger_joint = joint_positions[-6]
        self.joint_states = [finger_joint, right_outer_knuckle_joing]

    def update_contact_state(
        self,
        msg: Float32MultiArray
    ):
        """
        Get simulated Robotiq gripper contact state.

        ur5_contact_publisher topic is published by the custom Isaac Sim
        action graph node called ROS2ContactPublish.

        Parameters
        ----------
        msg : Float32MultiArray
            Float32MultiArray message from the /ur5_contact_publisher topic.
            The array data contains the following values in order:
            - force applied to the gripper left gripper pad (float)
            - force applied to the gripper right gripper pad (float)
            - Left gripper contact detection (bool, True if contact)
            - Right gripper contact detection (bool, True if contact)

        """
        self.contact_state = msg.data

    async def execute_callback(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        """
        Handle a new goal trajectory command.

        Parameters
        ----------
        goal_handle : rclpy.action.server.ServerGoalHandle
            ActionServer goal handle

        Returns
        -------
        FollowJointTrajectory.Result
            Result of the trajectory execution

        """
        if not self.trajectory_in_execution:
            self.get_logger().info('[GRIPPER] Executing goal...')

            contact_report = self.contact_state
            grippers_in_contact = contact_report[2] and contact_report[3]
            self.gripper_initially_in_contact = grippers_in_contact

            self.init_trajectory_gripper()
            initialized_correctly, msg =\
                self.initialize_trajectory(goal_handle)
            if initialized_correctly:
                result, total_time_from_init =\
                    await self.execute_trajectory(goal_handle)
                self.get_logger().info(
                    "[GRIPPER] Trajectory executed "
                    f"in {total_time_from_init:.2f} seconds")
                goal_handle.succeed()
                return result
            else:
                result = FollowJointTrajectory.Result()
                result._error_code = result.INVALID_GOAL
                result._error_string = msg
                return result

        # set goal_handle as not accepted
        self.get_logger().warn("[GRIPPER] Trajectory already in execution")
        result = FollowJointTrajectory.Result()
        result._error_code = result.INVALID_GOAL
        result._error_string =\
            "[GRIPPER] Failed. A trajectory is already in execution"
        return result

    def init_trajectory_gripper(self):
        """Initialize a new target trajectory."""
        self.actual_joint_state = copy.deepcopy(self.joint_states)

        print("\n-------- UR5 Gripper -------- ")
        print(f"Actual joint states (UR5 Gripper): {self.actual_joint_state}")
        print("----------------------- \n")

        self.time_t0 = time.time()

    def initialize_trajectory(
        self,
        received_goal_handle: rclpy.action.server.ServerGoalHandle
    ):
        """
        Handle a new goal trajectory command.

        Parameters
        ----------
        received_goal_handle : rclpy.action.server.ServerGoalHandle
            ActionServer goal handle

        Returns
        -------
        bool
            True if the trajectory was initialized
        str
            Error message

        """
        self.get_logger().info("[GRIPPER] Received goal")

        self.received_goal_handle = received_goal_handle
        if not tc.trajectory_is_finite(received_goal_handle.request.trajectory):
            error_msg = ("[GRIPPER] Trajectory not executed."
                         " Received a goal with infinites or NaNs")
            self.get_logger().error(error_msg)
            received_goal_handle.abort()
            return False, error_msg

        if self.actual_joint_state is None:
            error_msg = "[GRIPPER] No joint state received yet"
            self.get_logger().error(error_msg)
            received_goal_handle.abort()
            return False, error_msg

        self.trajectory_position_list = [self.actual_joint_state]
        self.time_from_start_list = [0.0]
        for point in received_goal_handle.request.trajectory.points:
            self.trajectory_position_list.append(point.positions)
            self.time_from_start_list.append(point.time_from_start.sec)

        self.timout = self.time_from_start_list[-1] + 1.0

        self.get_logger().info("[GRIPPER] Initializing cubic trajectory")
        self.init_interp_cubic()

        return True, ""

    def sample_trajectory(self, first_point=False):
        """
        Return (q, qdot, qddot) for sampling the JointTrajectory at time t.

        The time t is the time since the trajectory was started.

        Parameters
        ----------
        first_point : bool, optional
            If True, return the first point of the trajectory, by default False

        """
        if first_point:
            return self.actual_joint_state

        # Last point
        if (time.time() - self.time_t0) >= self.time_from_start_list[-1]:
            return self.trajectory_position_list[-1], [0.0]*self.n_joints

        return self.update_interp_cubic(time.time() - self.time_t0)

    def init_interp_cubic(self):
        """Initialize the cubic interpolation."""
        q0 = self.trajectory_position_list[0]
        qf = self.trajectory_position_list[-1]
        v0 = [0.0, 0.0]
        vf = [0.0, 0.0]
        t0 = self.time_from_start_list[0]
        tf = self.time_from_start_list[-1]
        a = [0.0]*self.n_joints
        for i in range(self.n_joints):
            q0_joint_i = q0[i]
            v0_joint_i = v0[i]
            qf_joint_i = qf[i]
            vf_joint_i = vf[i]
            b = np.array(
                [q0_joint_i, v0_joint_i, qf_joint_i, vf_joint_i]).transpose()
            m = np.array([[1, t0, t0**2,   t0**3],
                          [0,  1,  2*t0, 3*t0**2],
                          [1, tf, tf**2,   tf**3],
                          [0,  1,  2*tf, 3*tf**2]])
            a[i] = np.linalg.inv(m).dot(b)
        self.matrix_a = a

    def update_interp_cubic(self, t: float):
        """
        Interpolate a cubic polynomial trajectory.

        Parameters
        ----------
        t : float
            Time since the trajectory was started

        """
        pos_points = [0.0]*self.n_joints
        vel_points = [0.0]*self.n_joints
        a = self.matrix_a
        for j in range(self.n_joints):
            pos_points[j] = a[j][0] + a[j][1]*t + a[j][2]*t**2 + a[j][3]*t**3
            vel_points[j] = a[j][1] + 2*a[j][2]*t + 3*a[j][3]*t**2

        return pos_points, vel_points

    async def execute_trajectory(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ):
        """
        Update the trajectory command to the robot.

        Parameters
        ----------
        goal_handle : rclpy.action.server.ServerGoalHandle
            ActionServer goal handle

        Returns
        -------
        FollowJointTrajectory.Result
            Result of the trajectory execution

        """
        self.get_logger().info("[GRIPPER] Executing trajectory")
        result = FollowJointTrajectory.Result()
        result._error_code = result.INVALID_GOAL
        if goal_handle is not None:
            now = time.time()
            total_time_from_init = now - self.time_t0
            in_time = total_time_from_init <= self.timout
            position_in_tol = tc.within_tolerance(
                self.joint_states,
                self.trajectory_position_list[-1],
                [self.ros_parameters['trajectory_tolerance_error']] * self.n_joints)
            terminated_by_contact = False
            self.trajectory_in_execution = True

            while not position_in_tol and in_time:
                position, velocity = self.sample_trajectory()

                for i in range(self.n_joints):
                    position[i] = self.joint_directions_close[i] * \
                        abs(position[i])

                contact_report = self.contact_state
                grippers_in_contact = contact_report[2] and contact_report[3]
                grippers_force_above_threshold =\
                    contact_report[0] > self.ros_parameters['force_threshold'] \
                    and contact_report[1] > self.ros_parameters['force_threshold']

                if grippers_in_contact and grippers_force_above_threshold \
                        and not self.gripper_initially_in_contact:
                    self.get_logger().info("[GRIPPER] Grippers in contact. "
                                           f"Values: {contact_report}")
                    terminated_by_contact = True
                    break

                self.actual_trajectory_to_send.header.stamp =\
                    self.get_clock().now().to_msg()
                self.actual_trajectory_to_send.name = self.joint_names
                self.actual_trajectory_to_send.position = position
                self.actual_trajectory_to_send.velocity = velocity

                self.robot_isaac_pub.publish(
                    self.actual_trajectory_to_send)

                now = time.time()
                total_time_from_init = now - self.time_t0
                in_time = total_time_from_init <= self.timout
                position_in_tol = tc.within_tolerance(
                    self.joint_states,
                    self.trajectory_position_list[-1],
                    [self.ros_parameters['trajectory_tolerance_error']] * self.n_joints)

            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = self.joint_names
            feedback.desired.positions = self.trajectory_position_list[-1]
            feedback.actual.positions = self.joint_states
            feedback.error.positions = [
                a - b for a, b in zip(self.trajectory_position_list[-1], self.joint_states)]
            self.received_goal_handle.publish_feedback(feedback)
            self.get_logger().info("[GRIPPER] Feedback:")
            self.get_logger().info(f"\tDesired: {feedback.desired.positions}")
            self.get_logger().info(f"\tActual: {feedback.actual.positions}")
            self.get_logger().info(f"\tError: {feedback.error.positions}")

            if position_in_tol or terminated_by_contact:
                result._error_code = result.SUCCESSFUL
                result._error_string = "Success"
                self.received_goal_handle = None
            else:
                result._error_string =\
                    ("[GRIPPER] Failed. Joint states not in tolerance. Time exceeded.")
                self.get_logger().warn(
                    f"[GRIPPER] [TIMEOUT] {total_time_from_init} seconds")

            self.trajectory_in_execution = False
            return result, total_time_from_init


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    ur5_gripper_server = UR5GripperTrajController()
    executor = MultiThreadedExecutor()
    executor.add_node(ur5_gripper_server)

    try:
        ur5_gripper_server.get_logger().info(
            '[GRIPPER] Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        ur5_gripper_server.get_logger().info(
            '[GRIPPER] Keyboard interrupt, shutting down.\n')
    ur5_gripper_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
