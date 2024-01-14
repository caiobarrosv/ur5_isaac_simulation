#!/usr/bin/env python3
import copy
import math
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

from ur5_isaac_simulation.helper_functions import trajectory_check as tc
from ur5_isaac_simulation.helper_functions.load_ros_parameters import \
    get_ros_parameters


class UR5TrajController(Node):
    """Create and handle the action 'follow_joint_trajectory' server."""

    ur5_joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]

    def __init__(self):
        super().__init__('ur5_controller_server')

        ###############################
        # ROS PARAMETERS
        ###############################
        self.ros_parameters, declared_parameters =\
            get_ros_parameters("ur5_controller_server")
        self.declare_parameters(namespace='',
                                parameters=declared_parameters)
        self.get_logger().info("Parameters:")
        for param, value in declared_parameters:
            self.get_logger().info(f"\t[UR5] {param}: {value}")
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

        self.robotserver = ActionServer(
            self,
            FollowJointTrajectory,
            "ur5/follow_joint_trajectory",
            self.execute_callback,
            callback_group=client_cb_group)
        self.get_logger().info('ur5_controller_server [ON]!')

        self.actual_joint_state = []
        self.trajectory_position_list = []
        self.trajectory_velocity_list = []
        self.trajectory_acceleration_list = []
        self.time_from_start_list = []
        self.time_t0 = 0.0
        self.matrix_a = None
        self.actual_trajectory_to_send = JointState()
        self.received_goal_handle = None
        self.succeed = False
        self.trajectory_in_execution = False

    def parameters_callback(self, params):
        """Update ROS2 parameters according to the config/params.yaml file"""
        for param in params:
            if param.name == "home_joint_array":
                if param.type_ in [Parameter.Type.DOUBLE_ARRAY]:
                    self.ros_parameters['home_joint_array'] = list(param.value)
                else:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def update_joint_state(
        self,
        msg: JointState
    ):
        """Get simulated UR5 joint angles in the following order:

        shoulder_pan_joint, shoulder_lift_joint, elbow_joint
        wrist_1_joint, wrist_2_joint, wrist_3_joint

        Parameters
        ----------
        msg : JointState
            JointState message from the /joint_states topic

        """
        self.joint_states = msg.position[0:6]

    async def execute_callback(self, goal_handle):
        """Handle a new goal trajectory command.

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
            self.get_logger().info('[UR5] Executing goal...')

            self.init_trajectory_robot()
            initialized_correctly, msg =\
                self.initialize_trajectory(goal_handle)
            if initialized_correctly:
                result, total_time_from_init =\
                    await self.execute_trajectory(goal_handle)
                self.get_logger().info(
                    "[UR5] Trajectory executed "
                    f"in {total_time_from_init:.2f} seconds")
                goal_handle.succeed()
                return result
            else:
                result = FollowJointTrajectory.Result()
                result._error_code = result.INVALID_GOAL
                result._error_string = msg
                return result

        # Set goal state

        # set goal_handle as not accepted
        self.get_logger().warn("[UR5] Trajectory already in execution")
        result = FollowJointTrajectory.Result()
        result._error_code = result.INVALID_GOAL
        result._error_string =\
            "[UR5] Failed. A trajectory is already in execution"
        return result

    def init_trajectory_robot(self):
        """Initialize a new target trajectory."""
        self.actual_joint_state = copy.deepcopy(self.joint_states)

        print("\n-------- UR5 -------- ")
        print(f"Actual joint states (UR5): {self.actual_joint_state}")
        print("----------------------- \n")

        self.time_t0 = time.time()

    def initialize_trajectory(
        self,
        received_goal_handle: rclpy.action.server.ServerGoalHandle
    ):
        """Handle a new goal trajectory command.

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
        self.get_logger().info("[UR5] Received goal")

        self.received_goal_handle = received_goal_handle
        if not tc.trajectory_is_finite(received_goal_handle.request.trajectory):
            error_msg = ("[UR5] Trajectory not executed."
                         " Received a goal with infinites or NaNs")
            self.get_logger().error(error_msg)
            received_goal_handle.abort()
            return False, error_msg
        # Checks that the trajectory has velocities
        if not tc.has_velocities(received_goal_handle.request.trajectory):
            error_msg = "[UR5] Received a goal without velocities"
            self.get_logger().error(error_msg)
            received_goal_handle.abort()
            return False, error_msg

        if self.actual_joint_state is None:
            error_msg = "[UR5] No joint state received yet"
            self.get_logger().error(error_msg)
            received_goal_handle.abort()
            return False, error_msg

        self.trajectory_position_list = [self.actual_joint_state]
        self.trajectory_velocity_list = [[0.0]*6]
        self.trajectory_acceleration_list = [[0.0]*6]
        self.time_from_start_list = [0.0]
        for point in received_goal_handle.request.trajectory.points:
            self.trajectory_position_list.append(point.positions)
            self.trajectory_velocity_list.append(point.velocities)
            self.trajectory_acceleration_list.append(point.accelerations)
            self.time_from_start_list.append(point.time_from_start.sec)

        option = self.ros_parameters['trajectory_type']
        if option == 1:
            self.get_logger().info("[UR5] Initializing cubic trajectory")
            self.init_interp_cubic()
        elif option == 2:
            self.get_logger().info("[UR5] Initializing quintic trajectory")
            self.init_interp_quintic()
        elif option == 3:
            self.get_logger().info("[UR5] Initializing LSPB trajectory")
            self.init_lspb_trajectory()
        elif option == 4:
            self.get_logger().info("[UR5] Initializing minimum time trajectory")
            # Minimum Time Trajectories
            self.init_minimum_time_trajectory()

        # Considering that we have only 2 points
        actual_trajectory_to_send =\
            self.sample_trajectory(True)
        self.actual_trajectory_to_send.position = actual_trajectory_to_send[0]
        self.actual_trajectory_to_send.velocity = actual_trajectory_to_send[1]
        return True, ""

    def init_minimum_time_trajectory(self):
        """Initialize a minimum time trajectory."""
        self.q0 = self.trajectory_position_list[0]
        self.qf = self.trajectory_position_list[-1]
        self.alfa = [1.0]*6
        self.ts = [0.0]*6
        self.tf = [0.0]*6
        for i in range(6):
            if self.qf[i] - self.q0[i] > 0:
                self.alfa[i] = self.alfa[i]
            else:
                self.alfa[i] = -self.alfa[i]
            self.ts[i] = math.sqrt(
                abs((self.qf[i] - self.q0[i])/self.alfa[i]))
            self.tf[i] = 2*self.ts[i]
        self.time_from_start_list = [max(self.tf)]
        self.pos_atual = [0.0]*6
        self.vel_atual = [0.0]*6

    def init_lspb_trajectory(self):
        """Initialize a LSPB trajectory."""
        self.q0 = self.trajectory_position_list[0]
        self.qf = self.trajectory_position_list[-1]
        self.pos_atual = [0.0]*6
        self.vel_atual = [0.0]*6
        self.acc_atual = [0.0]*6
        self.tf = self.time_from_start_list[-1]
        self.tb = self.tf/3.0
        self.vtb = [0.0]*6
        for i in range(6):
            self.vtb[i] = 1.5*(self.qf[i] - self.q0[i])/self.tf

    def sample_trajectory(self, first_point=False):
        """Return (q, qdot, qddot) for sampling the JointTrajectory at time t,
        the time t is the time since the trajectory was started.

        Parameters
        ----------
        first_point : bool, optional
            If True, return the first point of the trajectory, by default False

        """
        if first_point:
            return self.actual_joint_state, [0.0]*6

        # Last point
        if (time.time() - self.time_t0) >= self.time_from_start_list[-1]:
            return self.trajectory_position_list[-1], [0.0]*6

        option = self.ros_parameters['trajectory_type']
        if option == 1:
            trajectory_method = self.update_interp_cubic
        elif option == 2:
            trajectory_method = self.update_interp_quintic
        elif option == 3:
            trajectory_method = self.update_lspb_trajectory
        elif option == 4:
            trajectory_method = self.update_minimum_time_traj
        return trajectory_method(time.time() - self.time_t0)

    def update_minimum_time_traj(self, t):
        """Update the minimum time trajectory.

        Parameters
        ----------
        t : float
            Time since the trajectory was started

        """
        for i in range(6):
            if t < self.ts[i]:
                self.pos_atual[i] = self.q0[i] + (self.alfa[i]/2)*t**2
                self.vel_atual[i] = (self.alfa[i])*t
            elif t >= self.ts[i] and t < self.tf[i]:
                self.pos_atual[i] = self.qf[i] \
                    - (self.alfa[i]*self.tf[i]**2)/2 \
                    + self.alfa[i]*self.tf[i]*t \
                    - (self.alfa[i]/2)*t**2
                self.vel_atual[i] = self.alfa[i]*self.tf[i] - self.alfa[i]*t

        return self.pos_atual, self.vel_atual

    def init_interp_cubic(self):
        """Interpolate a cubic polynomial trajectory"""
        q0 = self.trajectory_position_list[0]
        qf = self.trajectory_position_list[-1]
        v0 = self.trajectory_velocity_list[0]
        vf = self.trajectory_velocity_list[-1]
        t0 = self.time_from_start_list[0]
        tf = self.time_from_start_list[-1]
        a = [0.0]*6
        for i in range(6):
            q0_joint_i = q0[i]
            v0_joint_i = v0[i]
            qf_joint_i = qf[i]
            vf_joint_i = vf[i]
            # print("Junta: {} | q0: {:.2f} | qf: {:.2f}".format(i, q0, qf))
            b = np.array(
                [q0_joint_i, v0_joint_i, qf_joint_i, vf_joint_i]).transpose()
            m = np.array([[1, t0, t0**2,   t0**3],
                          [0,  1,  2*t0, 3*t0**2],
                          [1, tf, tf**2,   tf**3],
                          [0,  1,  2*tf, 3*tf**2]])
            a[i] = np.linalg.inv(m).dot(b)
        self.matrix_a = a

    def update_interp_cubic(self, t):
        """Interpolate a cubic polynomial trajectory

        Parameters
        ----------
        t : float
            Time since the trajectory was started

        """
        pos_points, vel_points = [0.0]*6, [0.0]*6
        a = self.matrix_a
        for j in range(6):
            pos_points[j] = a[j][0] + a[j][1]*t + a[j][2]*t**2 + a[j][3]*t**3
            vel_points[j] = a[j][1] + 2*a[j][2]*t + 3*a[j][3]*t**2

        return pos_points, vel_points

    def init_interp_quintic(self):
        """Interpolate a quintic polynomial trajectory"""
        init_vel = init_acc = end_vel = end_acc = 0
        # Tempo do primeiro ponto = 0
        init_t = self.time_from_start_list[0]
        # Tempo do ultimo ponto da trajetoria
        end_t = self.time_from_start_list[-1]
        joint_angles = [0.0]*6

        for i in range(6):
            # First joint i position
            init_joint_i = self.trajectory_position_list[0][i]
            # Last joint i position
            end_joint_i = self.trajectory_position_list[-1][i]
            mat_b = np.array([init_joint_i, init_vel, init_acc,
                              end_joint_i, end_vel, end_acc]).transpose()
            mat_m = np.array([[1, init_t, init_t**2,
                               init_t**3, init_t**4, init_t**5],
                              [0, 1, 2*init_t,
                               3*init_t**2, 4*init_t**3, 5*init_t**4],
                              [0, 0, 2,
                               6*init_t, 12*init_t**2, 20*init_t**3],
                              [1, end_t, end_t**2,
                               end_t**3, end_t**4, end_t**5],
                              [0, 1, 2*end_t,
                               3*end_t**2, 4*end_t**3,  5*end_t**4],
                              [0, 0, 2,
                               6*end_t, 12*end_t**2, 20*end_t**3]])
            # Interpolated joint i position
            joint_angles[i] = np.linalg.inv(mat_m).dot(mat_b)
        self.matrix_a = joint_angles

    def update_interp_quintic(
        self,
        t: float
    ):
        """Interpolate a quintic polynomial trajectory

        Parameters
        ----------
        t : float
            Time since the trajectory was started

        Returns
        -------
        list
            Actual position
        list
            Actual velocity

        """
        pos_points, vel_points, acc_points = [0.0]*6, [0.0]*6, [0.0]*6
        a = self.matrix_a
        for j in range(6):
            pos_points[j] = a[j][0] + a[j][1]*t + a[j][2] * \
                t**2 + a[j][3]*t**3 + a[j][4]*t**4 + a[j][5]*t**5
            vel_points[j] = a[j][1] + 2*a[j][2]*t + 3 * \
                a[j][3]*t**2 + 4*a[j][4]*t**3 + 5*a[j][5]*t**4
            acc_points[j] = 2*a[j][2] + 6*a[j][3] * \
                t + 12*a[j][4]*t**2 + 20*a[j][5]*t**3

        return pos_points, vel_points

    def update_lspb_trajectory(
        self,
        t: float
    ):
        """Update the LSPB trajectory.

        Parameters
        ----------
        t : float
            Time since the trajectory was started

        Returns
        -------
        list
            Actual position
        list
            Actual velocity

        """
        q0 = self.trajectory_position_list[0]
        qf = self.trajectory_position_list[-1]

        for i in range(6):
            alfa = self.vtb[i] / self.tb
            if t <= self.tb:
                self.pos_atual[i] = q0[i] + (alfa/2)*t**2
                self.vel_atual[i] = (alfa)*t
            elif t > self.tb and t <= (self.tf - self.tb):
                self.pos_atual[i] = (
                    qf[i] + q0[i] - self.vtb[i]*self.tf)/2 + self.vtb[i]*t
                self.vel_atual[i] = self.vtb[i]
            elif t > (self.tf - self.tb) and t <= self.tf:
                self.pos_atual[i] = qf[i] - alfa*self.tf**2 / \
                    2 + alfa*self.tf*t - alfa/2*t**2
                self.vel_atual[i] = alfa*self.tf - alfa*t

        return self.pos_atual, self.vel_atual

    async def execute_trajectory(
        self,
        goal_handle: rclpy.action.server.ServerGoalHandle
    ):
        """Update the trajectory command to the robot.

        Parameters
        ----------
        goal_handle : rclpy.action.server.ServerGoalHandle
            ActionServer goal handle

        Returns
        -------
        FollowJointTrajectory.Result
            Result of the trajectory execution

        """
        self.get_logger().info("[UR5] Executing trajectory")
        result = FollowJointTrajectory.Result()
        result._error_code = result.INVALID_GOAL
        if goal_handle is not None:
            now = time.time()
            total_time_from_init = now - self.time_t0
            in_time = total_time_from_init <= self.ros_parameters['trajectory_timout']
            position_in_tol = tc.within_tolerance(
                self.joint_states,
                self.trajectory_position_list[-1],
                [self.ros_parameters['trajectory_tolerance_error']] * 6)

            self.trajectory_in_execution = True
            while not position_in_tol and in_time:
                position, velocity = self.sample_trajectory()
                self.actual_trajectory_to_send.header.stamp =\
                    self.get_clock().now().to_msg()
                self.actual_trajectory_to_send.name = self.ur5_joint_names
                self.actual_trajectory_to_send.position = position
                self.actual_trajectory_to_send.velocity = velocity

                self.robot_isaac_pub.publish(
                    self.actual_trajectory_to_send)

                now = time.time()
                total_time_from_init = now - self.time_t0
                in_time = total_time_from_init <= self.ros_parameters['trajectory_timout']
                position_in_tol = tc.within_tolerance(
                    self.joint_states,
                    self.trajectory_position_list[-1],
                    [self.ros_parameters['trajectory_tolerance_error']] * 6)

            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = self.ur5_joint_names
            feedback.desired.positions = self.trajectory_position_list[-1]
            feedback.actual.positions = self.joint_states
            feedback.error.positions = [
                a - b for a, b in zip(self.trajectory_position_list[-1], self.joint_states)]
            self.get_logger().info("[UR5] Feedback:")
            self.get_logger().info(f"\tDesired: {feedback.desired.positions}")
            self.get_logger().info(f"\tActual: {feedback.actual.positions}")
            self.get_logger().info(f"\tError: {feedback.error.positions}")
            self.received_goal_handle.publish_feedback(feedback)

            if position_in_tol:
                result._error_code = result.SUCCESSFUL
                result._error_string = "Success"
                self.received_goal_handle = None
            else:
                result._error_string =\
                    ("[UR5] Failed. Joint states not in tolerance. Time exceeded.")
                self.get_logger().warn(
                    f"[UR5] [TIMEOUT] {total_time_from_init} seconds")

            self.trajectory_in_execution = False
            return result, total_time_from_init


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    ur5_isaac_server = UR5TrajController()
    executor = MultiThreadedExecutor()
    executor.add_node(ur5_isaac_server)

    try:
        ur5_isaac_server.get_logger().info(
            '[UR5] Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        ur5_isaac_server.get_logger().info(
            '[UR5] Keyboard interrupt, shutting down.\n')
    ur5_isaac_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
