# kinematics.py
"""
Forward Kinematics (FK) and Inverse Kinematics (IK) implementation using the Pybotics library.
"""
import numpy as np
import logging
from pybotics.robot import Robot
from pybotics.tool import Tool


def create_robot_from_dh(dh_params):
    """
    Creates a Pybotics Robot object from a list of DH parameters.
    """
    # Pybotics expects DH parameters in a specific format.
    # The order is [alpha, a, theta, d]
    # The input dh_params is a list of dicts: [{'alpha': ..., 'a': ..., 'theta': ..., 'd': ...}]
    dh_matrix = np.array([[p["alpha"], p["a"], p["theta"], p["d"]] for p in dh_params])
    robot = Robot.from_parameters(dh_matrix)
    return robot


def forward_kinematics(robot, joint_angles_deg):
    """
    Calculates FK using a Pybotics Robot object.
    """
    joint_angles_rad = np.radians(joint_angles_deg)
    transform_matrix = robot.fk(joint_angles_rad)
    position = transform_matrix[:3, 3]

    # Extract RPY from rotation matrix
    R = transform_matrix[:3, :3]
    pitch = np.arcsin(-R[2, 0])
    if abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        yaw = 0
        roll = np.arctan2(-R[1, 2], R[1, 1])
    orientation_rpy = np.degrees([roll, pitch, yaw])

    return tuple(position), tuple(orientation_rpy)


def inverse_kinematics(robot, target_pose, initial_guess_deg=None):
    """
    Calculates IK using the Pybotics Robot object.
    """
    target_position = target_pose[:3]
    target_orientation_rpy_deg = target_pose[3:]

    # Convert RPY to rotation matrix
    roll, pitch, yaw = np.radians(target_orientation_rpy_deg)
    Rx = np.array(
        [[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]]
    )
    Ry = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )
    Rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )
    target_orientation_matrix = Rz @ Ry @ Rx

    # Create the target transform matrix
    target_transform = np.eye(4)
    target_transform[:3, :3] = target_orientation_matrix
    target_transform[:3, 3] = target_position

    # Use the ik method from Pybotics
    if initial_guess_deg is not None:
        initial_guess_rad = np.radians(initial_guess_deg)
    else:
        initial_guess_rad = None

    try:
        ik_solution_rad = robot.ik(target_transform, q=initial_guess_rad)
        if ik_solution_rad is not None:
            return np.degrees(ik_solution_rad).tolist()
        else:
            logging.warning("Pybotics IK failed to find a solution.")
            return None
    except Exception as e:
        logging.error(f"Error during Pybotics IK calculation: {e}")
        return None
