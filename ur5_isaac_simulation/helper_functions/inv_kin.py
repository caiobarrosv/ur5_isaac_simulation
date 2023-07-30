#!/usr/bin/python3
"""Inverse kinematics of the UR5 robot."""
import cmath
from math import acos, atan2, cos, pi, sin, sqrt

import numpy as np
import numpy.linalg as linalg

from .transformations import (htm_rotation_around_x, htm_rotation_around_y,
                              htm_rotation_around_z, htm_translation)

MAT = np.matrix
TRANSL_PARAM_Z = np.array([0.089159, 0, 0, 0, 0.10915, 0.09465, 0.0823])
TRANSL_PARAM_X = np.array([0, 0, -0.425, -0.39225, 0, 0, 0])
ROT_PARAM_X = np.array([0, np.pi/2, 0, 0, 0, 0, -np.pi/2])


def htm_base_link_inertia_to_shoulder_link(theta1: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from base_link_inertia to shoulder_link

    Parameters
    ----------
    theta1 : float
        Joint angle of the shoulder_pan_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from base_link_inertia
        to shoulder_link

    """
    translation_z = htm_translation([0, 0, TRANSL_PARAM_Z[0]])
    rotation_z = htm_rotation_around_z(theta1)
    # We post multiply rotation_z because
    # shoulder_link is rotating around shoulder_link z axis
    # and not around base_link_inertia z axis
    htm_01 = translation_z * rotation_z
    return htm_01


def htm_shoulder_link_to_upper_arm_link(theta2: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from shoulder_link to upper_arm_link

    Parameters
    ----------
    theta2 : float
        Joint angle of the shoulder_lift_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from shoulder_link
        to upper_arm_link

    """
    rotation_x = htm_rotation_around_x(ROT_PARAM_X[1])
    rotation_z = htm_rotation_around_z(theta2)
    # We post multiply rotation_z because
    # upper_arm_link is rotating around upper_arm_link z axis
    # and not around shoulder_link z axis
    htm_12 = rotation_x * rotation_z
    return htm_12


def htm_upper_arm_link_to_forearm_link(theta3: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from upper_arm_link to forearm_link

    Parameters
    ----------
    theta3 : float
        Joint angle of the elbow_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from upper_arm_link
        to forearm_link

    """
    translation_x = htm_translation([TRANSL_PARAM_X[2], 0, 0])
    rotation_z = htm_rotation_around_z(theta3)
    htm_23 = translation_x * rotation_z
    return htm_23


def htm_forearm_link_to_forearm_link_x() -> np.matrix:
    """Compute the homogeneous transformation matrix
    from forearm_link to forearm_link_x

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from forearm_link
        to forearm_link_x

    """
    htm_34 = htm_translation([TRANSL_PARAM_X[3], 0, 0])
    return htm_34


def htm_forearm_link_x_to_wrist_1_link(theta4: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from forearm_link_x to wrist_1_link

    Parameters
    ----------
    theta4 : float
        Joint angle of the wrist_1_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from forearm_link_x
        to wrist_1_link

    """
    translation_z = htm_translation([0, 0, TRANSL_PARAM_Z[4]])
    rotation_z = htm_rotation_around_z(theta4)
    htm_45 = translation_z * rotation_z
    return htm_45


def htm_wrist_1_link_to_wrist_2_link(theta5: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from wrist_1_link to wrist_2_link

    Parameters
    ----------
    theta5 : float
        Joint angle of the wrist_2_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from wrist_1_link
        to wrist_2_link

    """
    rotation_x = htm_rotation_around_x(ROT_PARAM_X[5])
    translation_z = htm_translation([0, 0, TRANSL_PARAM_X[5]])
    rotation_z = htm_rotation_around_z(theta5)
    htm_56 = rotation_x * translation_z * rotation_z
    return htm_56


def htm_wrist_2_link_to_wrist_3_link(theta6: float) -> np.matrix:
    """Compute the homogeneous transformation matrix
    from wrist_2_link to wrist_3_link

    Parameters
    ----------
    theta6 : float
        Joint angle of the wrist_3_joint in radians

    Returns
    -------
    np.matrix
        Homogeneous transformation matrix from wrist_2_link
        to wrist_3_link

    """
    rotation_x = htm_rotation_around_x(ROT_PARAM_X[-1])
    translation_z = htm_translation([0, 0, TRANSL_PARAM_Z[-1]])
    rotation_z = htm_rotation_around_z(theta6)
    htm_67 = rotation_x * translation_z * rotation_z
    return htm_67


def get_theta1(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get set of solutions for theta1 angle.

    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the end-effector desired pose
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta1 angles

    """
    # position_link_89 is a column vector with the position of
    # the wrist_2_link in respect to the base_link_inertia
    position_link_89 =\
        desired_pose_07 * MAT([0, 0, -TRANSL_PARAM_Z[6], 1]).T \
        - MAT([0, 0, 0, 1]).T

    # beta is the angle between the projection in the xy plane of the
    # wrist_2_link and the x axis of the base_link_inertia
    beta = atan2(position_link_89[1, 0], position_link_89[0, 0])

    # Magnitude of the projection of the wrist_2_link in the xy plane
    position_wrist_2_xy = sqrt(position_link_89[1, 0]**2 +
                               position_link_89[0, 0]**2)

    if TRANSL_PARAM_Z[4] > position_wrist_2_xy:
        raise ValueError("d4 cannot be higher than position_wrist_2_xy."
                         "No solution for theta1")

    gamma = acos(TRANSL_PARAM_Z[4] / position_wrist_2_xy)

    # Be careful, shoulder right or left does not mean the direction of the
    # shoulder joint. It depends on shoulder_lift_joint angle and elbow_joint
    # angle. In other words, there is no rule to know if the shoulder is right
    # or left. It depends on the other joints. Just analyze the workspace of
    # the robot to know if the shoulder is right or left in your case.

    # Shoulder left or right (depends on shoulder_lift_joint and elbow_joint)
    solutions[0, 0:4] = pi/2 + beta + gamma
    # Shoulder left or right (depends on shoulder_lift_joint and elbow_joint)
    # For the Laboratory of Robotics at UFBA, keep the z axis of upper_arm_link
    # pointing to the direction opposite to the laboratory entrance or
    # (pi/2 + beta - gamma)
    solutions[0, 4:8] = pi/2 + beta - gamma

    return solutions


def get_theta5(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get set of solutions for theta5 angle.

    Recommended interval for theta5 at the Laboratory of Robotics at UFBA
    if the task is to pick an object from the table or inside the printer:
    [0, pi]

    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta5 angles

    """
    wrist_up_or_down_configs = [0, 4]  # wrist up or down
    for config in wrist_up_or_down_configs:
        # Theta1 can be used since it is already calculated
        htm_link_01 = htm_base_link_inertia_to_shoulder_link(
            solutions[0, config])
        # We assume that theta2 is still zero since it is not calculated yet
        htm_link_12 = htm_shoulder_link_to_upper_arm_link(0)
        htm_link_02 = htm_link_01 * htm_link_12
        htm_link_20 = linalg.inv(htm_link_02)
        htm_link_27 = htm_link_20 * desired_pose_07

        acos_num = htm_link_27[2, 3]-TRANSL_PARAM_Z[4]
        acos_den = TRANSL_PARAM_Z[6]

        if acos_num > acos_den:
            raise ValueError("P16z - d4 cannot be higher than d6."
                             "\nIn other words, the z axis of wrist_3_link"
                             "cannot be parallel to the wrist_2_link,"
                             " wrist_1_link, forearm_link and upper_arm_link"
                             "z axis.")

        theta_5 = acos(acos_num/acos_den)

        # NOTE: For UR5 at the Laboratory of Robotics at UFBA
        # If you are picking an object from the table or inside the printer
        # use +theta_5

        # Solution 1
        # Cols [0, 1, 4, 5] of solutions
        solutions[4, config:config+2] = theta_5
        # Solution 2
        # Cols [2, 3, 6, 7] of solutions
        solutions[4, config+2:config+4] = -theta_5

    return solutions


def get_theta6(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get set of solutions for theta6 angle.

    Range of theta6: [-pi, pi]

    print(f"Solution: {solutions}")    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta6 angles

    """
    # theta6 is not well-defined when sin(theta5) = 0
    # or when T16(1,3), T16(2,3) = 0.
    configs = [0, 2, 4, 6]
    for config in configs:
        # # Theta1 can be used since it is already calculated
        htm_link_01 = htm_base_link_inertia_to_shoulder_link(
            solutions[0, config])
        # We assume that theta2 is still zero since it is not calculated yet
        htm_link_12 = htm_shoulder_link_to_upper_arm_link(0)
        htm_link_02 = htm_link_01 * htm_link_12
        htm_link_20 = linalg.inv(htm_link_02)
        htm_link_27 = htm_link_20 * desired_pose_07
        htm_link_72 = linalg.inv(htm_link_27)

        # MODIFIED CODE
        theta5 = solutions[4, config]
        sin_theta5 = sin(theta5)

        theta6 = atan2(-htm_link_72[1, 2] / sin_theta5,
                       htm_link_72[0, 2] / sin_theta5)
        solutions[5, config:config+2] = theta6

    return solutions


def get_theta3(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get set of solutions for theta3 angle.

    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta3 angles

    """
    configs = [0, 2, 4, 6]
    for config in configs:
        # Theta1 can be used since it is already calculated
        theta1 = solutions[0, config]
        htm_link_01 = htm_base_link_inertia_to_shoulder_link(theta1)
        # We assume that theta2 is still zero since it is not calculated yet
        htm_link_12 = htm_shoulder_link_to_upper_arm_link(0)
        htm_link_02 = htm_link_01 * htm_link_12
        htm_link_20 = linalg.inv(htm_link_02)
        htm_link_27 = htm_link_20 * desired_pose_07

        theta6 = solutions[5, config]
        htm_link_67 = htm_wrist_2_link_to_wrist_3_link(theta6)
        theta5 = solutions[4, config]
        htm_link_56 = htm_wrist_1_link_to_wrist_2_link(theta5)

        # wrist_1_link in the upper_arm_link
        htm_link_25 = htm_link_27 * linalg.inv(htm_link_56 * htm_link_67)

        # Position of forearm_link_x in the upper_arm_link
        position_24 = htm_link_25 * \
            MAT([0, 0, -TRANSL_PARAM_Z[4], 1]).T - MAT([0, 0, 0, 1]).T

        theta3 = cmath.acos((
            linalg.norm(position_24)**2
            - TRANSL_PARAM_X[2]**2 - TRANSL_PARAM_X[3]**2) /
            (2 * TRANSL_PARAM_X[2] * TRANSL_PARAM_X[3]))
        solutions[2, config] = theta3.real
        solutions[2, config+1] = -theta3.real

    return solutions


def get_theta2(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get set of solutions for theta2 angle.

    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta2 angles

    """
    configs = [0, 1, 2, 3, 4, 5, 6, 7]
    for config in configs:
        # Theta1 can be used since it is already calculated
        theta1 = solutions[0, config]
        htm_link_01 = htm_base_link_inertia_to_shoulder_link(theta1)

        # We assume that theta2 is still zero since it is not calculated yet
        # shoulder_lift_joint should be 0 at this point
        htm_link_12 = htm_shoulder_link_to_upper_arm_link(0)
        htm_link_02 = htm_link_01 * htm_link_12

        htm_link_20 = linalg.inv(htm_link_02)
        # wrist_3_link in the upper_arm_link
        htm_link_27 = htm_link_20 * desired_pose_07

        theta6 = solutions[5, config]
        htm_link_67 = htm_wrist_2_link_to_wrist_3_link(theta6)
        theta5 = solutions[4, config]
        htm_link_56 = htm_wrist_1_link_to_wrist_2_link(theta5)

        htm_link_25 = htm_link_27 * linalg.inv(htm_link_56 * htm_link_67)

        position_24 = htm_link_25 * \
            MAT([0, 0, -TRANSL_PARAM_Z[4], 1]).T - MAT([0, 0, 0, 1]).T

        gamma = atan2(position_24[1, 0], position_24[0, 0])
        theta3 = solutions[2, config]
        beta = atan2(TRANSL_PARAM_X[3]*sin(theta3),
                     TRANSL_PARAM_X[2] + TRANSL_PARAM_X[3]*cos(theta3))
        theta2 = gamma - beta
        solutions[1, config] = theta2

    return solutions


def get_theta4(
    solutions: np.array,
    desired_pose_07: np.array
) -> np.matrix:
    """Get theta4 angle

    Parameters
    ----------
    solutions : np.array
        Set of solutions for the inverse kinematics
    desired_pose_07 : np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug : bool, optional
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse
        kinematics, by default True

    Returns
    -------
    np.array
        New solution matrix containing theta4 angles

    """
    configs = [0, 1, 2, 3, 4, 5, 6, 7]
    for config in configs:
        # Theta1 can be used since it is already calculated
        theta1 = solutions[0, config]
        htm_link_01 = htm_base_link_inertia_to_shoulder_link(theta1)
        theta2 = solutions[1, config]
        htm_link_12 = htm_shoulder_link_to_upper_arm_link(theta2)
        htm_link_02 = htm_link_01 * htm_link_12
        htm_link_20 = linalg.inv(htm_link_02)
        htm_link_27 = htm_link_20 * desired_pose_07

        theta6 = solutions[5, config]
        htm_link_67 = htm_wrist_2_link_to_wrist_3_link(theta6)
        theta5 = solutions[4, config]
        htm_link_56 = htm_wrist_1_link_to_wrist_2_link(theta5)

        htm_link_25 = htm_link_27 * linalg.inv(htm_link_56 * htm_link_67)

        htm_link_34 = htm_forearm_link_to_forearm_link_x()
        theta3 = solutions[2, config]
        htm_link_23 = htm_upper_arm_link_to_forearm_link(theta3)
        htm_link_45 = linalg.inv(htm_link_34) * \
            linalg.inv(htm_link_23) * htm_link_25
        theta4 = atan2(htm_link_45[1, 0], htm_link_45[0, 0])

        solutions[3, config] = theta4

    return solutions


def inverse_kinematics(
    desired_pose_07: np.array,
    print_debug: bool = False,
    solution_index: int = 5
) -> np.matrix:
    """Get the UR5 joint angles from the wrist_3_link
    pose (position + orientation).

    5 is the best solution_index if you are using the UR5
    at Laboratory of Robotics at UFBA. It represents the robot
    in the elbow up and wrist up configuration.

    Parameters
    ----------
    desired_pose_07: np.array
        Transformation matrix of the wrist_3_link desired pose
        in the base_link_inertia frame
    print_debug: bool
        Set True if you want to print the joint angles while
        moving the robot to analyze/study the inverse kinematics
    solution_index: int
        Index of the best solution for the Laboratory of Robotics at UFBA

    Returns
    --------
    np.array:
        solutions is a matrix of shape (6, 8) in wich each column
        corresponds to a robot configuration such as elbow down,
        elbow up, wrist down, etc.

    """
    # 1 solution for each column of solutions
    solutions = MAT(np.zeros((6, 8)))

    # Get a set of theta1 angles
    solutions = get_theta1(solutions, desired_pose_07)

    # Get a set of theta5 angles
    solutions = get_theta5(solutions, desired_pose_07)

    # Get a set of theta6 angles
    solutions = get_theta6(solutions, desired_pose_07)

    # Get a set of theta3 angles
    solutions = get_theta3(solutions, desired_pose_07)

    # Get a set of theta2 angles
    solutions = get_theta2(solutions, desired_pose_07)

    # Get a set of theta2 angles
    solutions = get_theta4(solutions, desired_pose_07)

    if print_debug:
        solution_deg = np.degrees(solutions).astype(int)
        print(f"Inverse Kinematics Solutions [degrees|int]:\n{solution_deg}")

    solutions = np.array(solutions.real[:, solution_index]).reshape(6)
    return solutions


def debug_htm_matrices(angles: np.array):
    """Print the homogeneous transformation matrices.
    It is useful if you want to study the inverse kinematics.

    Parameters
    ----------
    angles : np.array
        Joint angles in radians

    """
    print("---------------")
    htm_01 = htm_base_link_inertia_to_shoulder_link(angles[0])
    print(f"htm_01: \n{np.round(htm_01, 3)}")

    htm_12 = htm_shoulder_link_to_upper_arm_link(angles[1])
    print(f"htm_12: \n{np.round(htm_12, 3)}")

    htm_02 = htm_01 * htm_12
    print(f"htm_02: \n{np.round(htm_02, 3)}")

    htm_20 = linalg.inv(htm_02)
    print(f"htm_20: \n{np.round(htm_20, 3)}")

    htm_23 = htm_upper_arm_link_to_forearm_link(angles[2])
    print(f"htm_23: \n{np.round(htm_23, 3)}")

    htm_34 = htm_forearm_link_to_forearm_link_x()
    print(f"htm_34: \n{np.round(htm_34, 3)}")

    htm_45 = htm_forearm_link_x_to_wrist_1_link(angles[3])
    print(f"htm_45: \n{np.round(htm_45, 3)}")

    htm_35 = htm_34 * htm_45
    print(f"htm_35: \n{np.round(htm_35, 3)}")

    htm_56 = htm_wrist_1_link_to_wrist_2_link(angles[4])
    print(f"htm_56: \n{np.round(htm_56, 3)}")

    htm_67 = htm_wrist_2_link_to_wrist_3_link(angles[5])
    print(f"htm_67: \n{np.round(htm_67, 3)}")

    htm_07 = htm_02 * htm_23 * htm_35 * htm_56 * htm_67
    print(f"htm_07: \n{np.round(htm_07, 3)}")


def main():
    """Main function"""
    # These are wrist_3_link rotation angles in the
    # base_link_inertia frame
    roll_angle_deg = 0
    pitch_angle_deg = 90
    yaw_angle_deg = 0

    rotation_around_x = htm_rotation_around_x(np.radians(roll_angle_deg))
    rotation_around_y = htm_rotation_around_y(np.radians(pitch_angle_deg))
    rotation_around_z = htm_rotation_around_z(np.radians(yaw_angle_deg))
    desired_pose = rotation_around_x * \
        rotation_around_y * \
        rotation_around_z

    # position vector is the desired position of the end effector
    # in the base_link_inertia frame
    position_vector = np.array([[0.5, 0.0, 0.5]]).T

    # set the desired pose to have the same position as the position vector
    desired_pose[:3, -1] = position_vector
    print(f"desired_pose: \n{desired_pose}")

    joint_angles = inverse_kinematics(desired_pose, solution_index=5)
    print(f"Joint angles: \n{joint_angles}")


if __name__ == "__main__":
    main()
