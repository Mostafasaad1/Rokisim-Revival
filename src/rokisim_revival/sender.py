from typing import List, Tuple, Optional, Dict
from .xml_parser import XMLRobotParser
from .kinematics import create_robot_from_dh, forward_kinematics, inverse_kinematics
import socket
import struct
import logging
from pybotics.robot import Robot


class RobotDefinition:
    """Holds robot definition data."""

    def __init__(self, name: str = "Unknown"):
        self.name: str = name
        self.joint_min_angles: List[float] = []
        self.joint_max_angles: List[float] = []
        self.dh_params: List[Dict[str, float]] = []
        self.axis_id_to_index: Dict[str, int] = {}

    def get_num_joints(self) -> int:
        return len(self.joint_min_angles)

    def get_joint_limits(self) -> Tuple[List[float], List[float]]:
        return self.joint_min_angles, self.joint_max_angles

    def get_active_joint_dh_parameters(self) -> List[Dict[str, float]]:
        active_dh = []
        sorted_items = sorted(self.axis_id_to_index.items(), key=lambda item: item[1])
        for axis_id, dh_index in sorted_items:
            if axis_id.startswith("Joint"):
                active_dh.append(self.dh_params[dh_index])
        return active_dh


class RoKiSimSender:
    """Handles communication with RoKiSim and kinematics."""

    def __init__(self, ip: str = "127.0.0.1", port: int = 2001):
        self.ip = ip
        self.port = port
        self.protocol_num_joints = 6
        self.buffer_size = 56
        self.buffer = bytearray(self.buffer_size)
        self.buffer[3] = self.protocol_num_joints
        self.buffer[7] = 1
        self.robot_definition = RobotDefinition()
        self._pybotics_robot: Optional[Robot] = None
        self._parser = XMLRobotParser()
        logging.info(f"Initialized for {ip}:{port}")

    def load_robot_definition(self, xml_file_path: str) -> None:
        name, min_angles, max_angles, dh_params, axis_index = self._parser.parse(
            xml_file_path
        )
        self.robot_definition = RobotDefinition(name)
        self.robot_definition.joint_min_angles = min_angles
        self.robot_definition.joint_max_angles = max_angles
        self.robot_definition.dh_params = dh_params
        self.robot_definition.axis_id_to_index = axis_index
        num_joints = self.robot_definition.get_num_joints()
        logging.info(f"Loaded {num_joints} joints for {name}")
        self._pybotics_robot = None  # Invalidate cache
        # Assumed completion for truncation: Rebuild Pybotics robot
        self._get_or_create_robot()

    def _get_or_create_robot(self) -> Optional[Robot]:
        if self._pybotics_robot is None:
            dh_params = self.robot_definition.get_active_joint_dh_parameters()
            if dh_params:
                self._pybotics_robot = create_robot_from_dh(dh_params)
        return self._pybotics_robot

    def validate_angles(self, angles: List[float]) -> bool:
        num_loaded = self.robot_definition.get_num_joints()
        if len(angles) != num_loaded:
            logging.warning(
                f"Invalid number of angles: {num_loaded} required, got {len(angles)}."
            )
            return False
        if num_loaded == 0:
            return True
        min_angles, max_angles = self.robot_definition.get_joint_limits()
        for i, angle in enumerate(angles):
            if not (min_angles[i] <= angle <= max_angles[i]):
                logging.warning(f"Angle {i+1} out of range.")
                return False
        return True

    def calculate_fk(
        self, joint_angles: List[float]
    ) -> Tuple[
        Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float]]
    ]:
        robot = self._get_or_create_robot()
        if robot is None or len(joint_angles) != 6:  # Hardcoded for 6 DOF
            return None, None
        return forward_kinematics(robot, joint_angles)

    def calculate_ik(
        self,
        target_pose: Tuple[float, ...],
        initial_guess: Optional[List[float]] = None,
    ) -> Optional[List[float]]:
        robot = self._get_or_create_robot()
        if robot is None or len(target_pose) != 6:
            return None
        return inverse_kinematics(robot, target_pose, initial_guess)

    def send_angles(self, angles: List[float]) -> None:
        num_loaded = self.robot_definition.get_num_joints()
        if len(angles) != num_loaded:
            raise ValueError(f"Expected {num_loaded} angles.")
        # Buffer packing logic (cleaned, with context manager for socket)
        self.buffer[8:] = b"\x00" * (self.buffer_size - 8)
        index = 8
        angles_to_send = (
            angles[: self.protocol_num_joints]
            if len(angles) > self.protocol_num_joints
            else angles
        )
        for angle in angles_to_send:
            angle_bytes = struct.pack(">d", float(angle))
            self.buffer[index : index + 8] = angle_bytes
            index += 8
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(2)
            sock.connect((self.ip, self.port))
            sock.sendall(self.buffer)
        logging.debug(f"Sent {len(angles_to_send)} joints")

    # Example usage remains, but moved to tests or examples
