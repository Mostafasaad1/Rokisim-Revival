import logging
from typing import Dict, List, Optional, Tuple

import numpy as np
from pybotics.robot import Robot


def create_robot_from_dh(dh_params: List[Dict[str, float]]) -> Robot:
    """Create Pybotics Robot from DH parameters."""
    dh_matrix = np.array([[p["alpha"], p["a"], p["theta"], p["d"]] for p in dh_params])
    return Robot.from_parameters(dh_matrix)


def forward_kinematics(
    robot: Robot, joint_angles_deg: List[float]
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """Calculate FK position and RPY orientation."""
    joint_angles_rad = np.radians(joint_angles_deg)
    transform_matrix = robot.fk(joint_angles_rad)
    position = tuple(transform_matrix[:3, 3])
    R = transform_matrix[:3, :3]
    pitch = np.arcsin(-R[2, 0])
    if abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        yaw = 0
        roll = np.arctan2(-R[1, 2] * np.sign(np.sin(pitch)), R[1, 1])
    orientation_rpy = tuple(np.degrees([roll, pitch, yaw]))
    return position, orientation_rpy


def inverse_kinematics(
    robot: Robot,
    target_pose: Tuple[float, ...],
    initial_guess_deg: Optional[List[float]] = None,
) -> Optional[List[float]]:
    """Calculate IK joint angles."""
    target_position = target_pose[:3]
    target_rpy_deg = target_pose[3:]
    roll, pitch, yaw = np.radians(target_rpy_deg)
    Rx = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0, sin(roll), cos(roll)]])
    Ry = np.array(
        [[cos(pitch), 0, sin(pitch)], [0, 1, 0], [-sin(pitch), 0, cos(pitch)]]
    )
    Rz = np.array([[cos(yaw), -sin(yaw), 0], [sin(yaw), cos(yaw), 0], [0, 0, 1]])
    target_orientation = Rz @ Ry @ Rx
    target_transform = np.eye(4)
    target_transform[:3, :3] = target_orientation
    target_transform[:3, 3] = target_position
    initial_guess_rad = np.radians(initial_guess_deg) if initial_guess_deg else None
    try:
        ik_rad = robot.ik(target_transform, q=initial_guess_rad)
        if ik_rad is not None:
            return np.degrees(ik_rad).tolist()
        else:
            logging.warning("IK failed to find solution.")
            return None
    except Exception as e:
        logging.error(f"IK error: {e}")
        return None
