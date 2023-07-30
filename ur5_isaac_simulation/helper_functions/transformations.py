from math import cos, sin

import numpy as np
import tf_transformations


def geometry_msg_pose_to_htm(geometry_msg):
    """Convert a geometry_msgs.transform message to a homogeneous transformation matrix.

    Parameters
    ----------
    geometry_msg: geometry_msgs.transform
        Pose message to convert

    Returns
    -------
    np.array:
        Homogeneous transformation matrix
    """
    position = geometry_msg.translation
    orientation = geometry_msg.rotation
    translation = np.array([position.x, position.y, position.z])
    rotation = np.array([orientation.x, orientation.y,
                         orientation.z, orientation.w])
    homogeneous_transformation = tf_transformations.quaternion_matrix(rotation)
    homogeneous_transformation[0:3, 3] = translation
    return homogeneous_transformation


def htm_rotation_around_x(angle: float) -> np.matrix:
    """Rotation matrix around x axis

    Parameters
    ----------
    angle : float
        Rotation angle in radians

    Returns
    -------
    np.matrix
        Rotation

    """
    return np.matrix([[1, 0, 0, 0],
                     [0, cos(angle), -sin(angle), 0],
                     [0, sin(angle), cos(angle), 0],
                     [0, 0, 0, 1]])


def htm_rotation_around_y(angle: float) -> np.matrix:
    """Rotation matrix around y axis

    Parameters
    ----------
    angle : float
        Rotation angle in radians

    Returns
    -------
    np.matrix
        Rotation

    """
    return np.matrix([[cos(angle), 0, sin(angle), 0],
                     [0, 1, 0, 0],
                     [-sin(angle), 0, cos(angle), 0],
                     [0, 0, 0, 1]])


def htm_rotation_around_z(angle: float) -> np.matrix:
    """Rotation matrix around z axis

    Parameters
    ----------
    angle : float
        Rotation angle in radians

    Returns
    -------
    np.matrix
        Rotation

    """
    return np.matrix([[cos(angle), -sin(angle), 0, 0],
                     [sin(angle), cos(angle), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def htm_translation(translation_vector: np.array) -> np.matrix:
    """Generate a homogeneous transformation matrix for translation

    Parameters
    ----------
    translation : np.array
        Translation vector [x, y, z]

    Returns
    -------
    np.matrix
        Translation

    """
    return np.matrix([[1, 0, 0, translation_vector[0]],
                     [0, 1, 0, translation_vector[1]],
                     [0, 0, 1, translation_vector[2]],
                     [0, 0, 0, 1]])


def get_desired_pose_htm(
    position: np.array,
    roll: float,
    pitch: float,
    yaw: float
):
    """Get the desired end effector pose in the base_link_inertia frame.

    Parameters
    ----------
    position : np.array. shape = (3,)
        Position of the end effector in the base_link_inertia frame
    roll : float
        Roll angle in degrees
    pitch : float
        Pitch angle in degrees
    yaw : float
        Yaw angle in degrees

    """
    rot_x = htm_rotation_around_x(np.radians(roll))
    rot_y = htm_rotation_around_y(np.radians(pitch))
    rot_z = htm_rotation_around_z(np.radians(yaw))
    desired_pose = rot_z * rot_y * rot_x

    # position vector is the desired position of the end effector
    # in the base_link_inertia frame
    position_vector = position.reshape(3, 1)

    # set the desired pose to have the same position as the position vector
    desired_pose[:3, -1] = position_vector

    return desired_pose
