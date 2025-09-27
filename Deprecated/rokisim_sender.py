# rokisim_sender.py
"""
Core module for sending joint angle data to RoKiSim and managing robot definitions.
Includes functionality to load robot definitions (joint limits, DH params) from XML using lxml.
Designed to be flexible regarding the number of joints defined in the XML,
though communication with RoKiSim is currently fixed for 6 joints based on the protocol.
Includes FK/IK capabilities using Pybotics.
"""
from lxml import etree
import socket
import struct
import logging
import kinematics  # Import the new kinematics module


class RobotDefinition:
    """
    Holds the definition of a robot loaded from an XML file.
    Includes joint limits and Denavit-Hartenberg (DH) parameters.
    """

    def __init__(self, name="Unknown"):
        self.name = name
        self.joint_min_angles = []
        self.joint_max_angles = []
        self.dh_params = []
        self.axis_id_to_index = {}

    def get_num_joints(self):
        """Returns the number of active joints loaded."""
        return len(self.joint_min_angles)

    def get_joint_limits(self):
        """Returns the min and max angles for the loaded joints."""
        return self.joint_min_angles, self.joint_max_angles

    def get_all_dh_parameters(self):
        """Returns the DH parameters for all axes found."""
        return self.dh_params

    def get_active_joint_dh_parameters(self):
        """Returns the DH parameters for the active joints."""
        active_dh = []
        sorted_items = sorted(self.axis_id_to_index.items(), key=lambda item: item[1])
        for axis_id, dh_index in sorted_items:
            if axis_id.startswith("Joint"):
                active_dh.append(self.dh_params[dh_index])
        return active_dh


class RoKiSimSender:
    """
    Handles sending joint angle data to RoKiSim via TCP.
    Can load robot definitions and perform FK/IK using Pybotics.
    """

    def __init__(self, ip="127.0.0.1", port=2001):
        """
        Initializes the sender.
        :param ip: IP address of the RoKiSim server.
        :param port: Port number of the RoKiSim server.
        """
        self.ip = ip
        self.port = port
        self.buffer_size = 56
        self.buffer = bytearray(self.buffer_size)
        self.protocol_num_joints = 6
        self.buffer[3] = self.protocol_num_joints
        self.buffer[7] = 1

        self.robot_definition = RobotDefinition()
        # Cache for the Pybotics Robot object
        self._pybotics_robot = None

        logging.info(f"RoKiSimSender initialized for {self.ip}:{self.port}")
        logging.info(f"Protocol expects {self.protocol_num_joints} joints.")

    def load_robot_definition(self, xml_file_path):
        """Loads robot definition from an XML file."""
        try:
            parser = etree.XMLParser(
                recover=True, no_network=True, resolve_entities=False
            )
            with open(xml_file_path, "r", encoding="utf-8") as f:
                content = f.read()
            import re

            # Fixed regex pattern for name attribute in robot_dk tag
            pattern = r"(<robot_dk\s+[^>]*?name=)([^\s>\"\']+)([\s>])"
            fixed_content = re.sub(pattern, r'\1"\2"\3', content, flags=re.DOTALL)
            root = etree.fromstring(fixed_content.encode("utf-8"), parser)

            logging.debug(f"Root tag: {root.tag}")

            # Check if the root itself is robot_dk or if it's a child
            if root.tag == "robot_dk":
                robot_dk = root
            else:
                robot_dk = root.find("robot_dk")

            if robot_dk is None:
                child_tags = [child.tag for child in root]
                logging.error(f"XML root children: {child_tags}")
                raise ValueError("XML does not contain 'robot_dk' element.")

            logging.debug(f"Found robot_dk element")
            logging.debug(f"robot_dk attributes: {robot_dk.attrib}")

            robot_name = robot_dk.get("name", "Unknown")
            self.robot_definition = RobotDefinition(name=robot_name)
            logging.info(f"Loading definition for robot: {self.robot_definition.name}")

            all_children_tags = [child.tag for child in robot_dk]
            logging.debug(f"robot_dk children tags: {all_children_tags}")
            axis_elements = robot_dk.findall("axis")
            logging.debug(f"Found {len(axis_elements)} axis elements using findall")

            for axis in axis_elements:
                axis_id = axis.get("id")
                logging.debug(f"Processing axis with id: '{axis_id}'")
                if axis_id:
                    try:
                        dh_entry = {
                            "alpha": float(axis.get("alpha", 0.0)),
                            "a": float(axis.get("a", 0.0)),
                            "theta": float(axis.get("theta", 0.0)),
                            "d": float(axis.get("d", 0.0)),
                        }

                        self.robot_definition.dh_params.append(dh_entry)
                        dh_index = len(self.robot_definition.dh_params) - 1
                        self.robot_definition.axis_id_to_index[axis_id] = dh_index

                        limsup_str = axis.get("limsup")
                        liminf_str = axis.get("liminf")
                        maxvalue_str = axis.get("maxvalue")
                        minvalue_str = axis.get("minvalue")

                        has_limits = False
                        max_limit = 180.0
                        min_limit = -180.0

                        if limsup_str is not None and liminf_str is not None:
                            max_limit = float(limsup_str)
                            min_limit = float(liminf_str)
                            has_limits = True
                            logging.debug(
                                f"  Using limsup/liminf: [{min_limit}, {max_limit}]"
                            )
                        elif maxvalue_str is not None and minvalue_str is not None:
                            max_limit = float(maxvalue_str)
                            min_limit = float(minvalue_str)
                            has_limits = True
                            logging.debug(
                                f"  Using maxvalue/minvalue: [{min_limit}, {max_limit}]"
                            )

                        if axis_id.startswith("Joint") and has_limits:
                            self.robot_definition.joint_min_angles.append(min_limit)
                            self.robot_definition.joint_max_angles.append(max_limit)
                            logging.debug(
                                f"  Added as active joint {len(self.robot_definition.joint_min_angles)}: {axis_id}"
                            )

                    except (ValueError, TypeError, AttributeError) as e:
                        logging.warning(f"Error parsing axis {axis_id}: {e}")

            num_loaded_joints = self.robot_definition.get_num_joints()
            logging.info(
                f"Loaded DH parameters for {len(self.robot_definition.dh_params)} axes."
            )
            logging.info(f"Loaded joint limits for {num_loaded_joints} joints.")

            if num_loaded_joints > 0:
                logging.debug(
                    f"Sample Limits for Joint1: Min={self.robot_definition.joint_min_angles[0]}, Max={self.robot_definition.joint_max_angles[0]}"
                )
            joint_dh_params = self.robot_definition.get_active_joint_dh_parameters()
            if joint_dh_params:
                logging.debug(f"Sample DH for first active joint: {joint_dh_params[0]}")

            if num_loaded_joints != self.protocol_num_joints:
                logging.warning(
                    f"Number of joints in XML ({num_loaded_joints}) does not match "
                    f"RoKiSim protocol expectation ({self.protocol_num_joints}). "
                )

            # Invalidate cached Pybotics Robot object as definition has changed
            self._pybotics_robot = None

        except etree.XMLSyntaxError as e:
            logging.error(
                f"Failed to parse XML file '{xml_file_path}' (lxml syntax error): {e}"
            )
            raise
        except FileNotFoundError:
            logging.error(f"XML file not found: '{xml_file_path}'")
            raise
        except Exception as e:
            logging.error(
                f"Unexpected error loading robot definition from '{xml_file_path}': {e}"
            )
            raise

    def get_loaded_robot_definition(self):
        """Returns the currently loaded RobotDefinition object."""
        return self.robot_definition

    def _get_or_create_robot(self):
        """Helper to get the Pybotics Robot object, creating it if necessary."""
        if self._pybotics_robot is None:
            try:
                dh_params = self.robot_definition.get_active_joint_dh_parameters()
                if len(dh_params) == self.protocol_num_joints:
                    self._pybotics_robot = kinematics.create_robot_from_dh(dh_params)
                    logging.debug("Created Pybotics Robot object for FK/IK.")
                else:
                    logging.error(
                        "Cannot create Pybotics Robot object: Incorrect number of DH parameters."
                    )
                    self._pybotics_robot = None
            except Exception as e:
                logging.error(f"Failed to create Pybotics Robot object: {e}")
                self._pybotics_robot = None
        return self._pybotics_robot

    def validate_angles(self, angles):
        """Validates joint angles against loaded limits."""
        num_loaded_joints = self.robot_definition.get_num_joints()
        if len(angles) != num_loaded_joints:
            logging.error(
                f"validate_angles: {num_loaded_joints} angles required, got {len(angles)}."
            )
            return False

        if num_loaded_joints == 0:
            logging.warning(
                "validate_angles: No joint limits loaded. Validation skipped."
            )
            return True

        min_angles, max_angles = self.robot_definition.get_joint_limits()
        for i in range(num_loaded_joints):
            if not (min_angles[i] <= angles[i] <= max_angles[i]):
                logging.warning(
                    f"Angle {i+1} ({angles[i]}) is out of range "
                    f"[{min_angles[i]}, {max_angles[i]}]"
                )
                return False
        return True

    def calculate_fk(self, joint_angles):
        """
        Calculates the Forward Kinematics using the loaded robot definition and Pybotics.
        :param joint_angles: List of joint angles.
        :return: Tuple (position, orientation_rpy) or (None, None) on error.
        """
        try:
            robot = self._get_or_create_robot()
            if robot is None:
                raise RuntimeError("Pybotics Robot object not initialized.")

            expected_dofs = 6  # Hardcode for our known robot

            if len(joint_angles) != expected_dofs:
                raise ValueError(
                    f"FK requires {expected_dofs} joint angles, got {len(joint_angles)}."
                )

            position, orientation_rpy = kinematics.forward_kinematics(
                robot, joint_angles
            )
            return position, orientation_rpy
        except Exception as e:
            logging.error(f"Error calculating FK: {e}")
            return None, None

    def calculate_ik(self, target_pose, initial_guess=None):
        """
        Calculates the Inverse Kinematics using the loaded robot definition and Pybotics.
        :param target_pose: Tuple (x, y, z, roll, pitch, yaw).
        :param initial_guess: Optional list of 6 joint angles.
        :return: List of joint angles or None on failure.
        """
        try:
            robot = self._get_or_create_robot()
            if robot is None:
                raise RuntimeError("Pybotics Robot object not initialized.")

            if len(target_pose) != 6:
                raise ValueError("IK target_pose must be (x, y, z, roll, pitch, yaw).")

            solution = kinematics.inverse_kinematics(
                robot, target_pose, initial_guess_deg=initial_guess
            )
            return solution
        except Exception as e:
            logging.error(f"Error calculating IK: {e}")
            return None

    def send_angles(self, angles):
        """Sends joint angles to RoKiSim."""
        num_loaded_joints = self.robot_definition.get_num_joints()
        num_to_send = len(angles)

        if num_to_send != num_loaded_joints:
            raise ValueError(
                f"Exactly {num_loaded_joints} joint angles are required based on loaded definition."
            )

        self.buffer[8:] = b"\x00" * (self.buffer_size - 8)
        index = 8

        if num_to_send <= self.protocol_num_joints:
            if num_to_send < self.protocol_num_joints:
                logging.warning(
                    f"Sending {num_to_send} angles, protocol expects {self.protocol_num_joints}. "
                    f"Remaining joints will be zero."
                )
            angles_to_send = angles
        else:
            logging.warning(
                f"Loaded definition has {num_to_send} joints, but protocol only supports "
                f"{self.protocol_num_joints}. Sending only the first {self.protocol_num_joints}."
            )
            angles_to_send = angles[: self.protocol_num_joints]

        for angle in angles_to_send:
            angle_bytes = struct.pack(">d", float(angle))
            if index + 8 <= self.buffer_size:
                self.buffer[index : index + 8] = angle_bytes
                index += 8
            else:
                logging.error("Buffer overflow prevented while packing angles.")
                raise ValueError("Buffer overflow while packing angles.")

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(2)
                sock.connect((self.ip, self.port))
                sock.sendall(self.buffer)
            logging.debug(
                f"Data sent successfully to {self.ip}:{self.port} ({len(angles_to_send)} joints)"
            )
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            logging.error(
                f"Network error sending data to RoKiSim ({self.ip}:{self.port}): {e}"
            )
            raise
        except Exception as e:
            logging.error(f"Unexpected error during send/close to RoKiSim: {e}")
            raise


# --- Example usage if run directly ---
if __name__ == "__main__":
    import time
    import numpy as np

    logging.basicConfig(
        level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    sender = RoKiSimSender(ip="127.0.0.1", port=2001)

    print("--- Test 1: Default robot definition ---")
    default_def = sender.get_loaded_robot_definition()
    print(f"Robot Name: {default_def.name}")
    print(f"Default Num Joints: {default_def.get_num_joints()}")

    # Create a simple test XML for demonstration
    test_xml_content = """<?xml version="1.0" encoding="UTF-8"?>
<robot_dk name="TestRobot">
    <axis id="Joint1" alpha="0" a="0" theta="0" d="290" limsup="165" liminf="-165"/>
    <axis id="Joint2" alpha="-90" a="0" theta="0" d="0" limsup="110" liminf="-110"/>
    <axis id="Joint3" alpha="0" a="270" theta="0" d="0" limsup="70" liminf="-110"/>
    <axis id="Joint4" alpha="-90" a="70" theta="0" d="302" limsup="160" liminf="-160"/>
    <axis id="Joint5" alpha="90" a="0" theta="0" d="0" limsup="120" liminf="-120"/>
    <axis id="Joint6" alpha="-90" a="0" theta="0" d="72" limsup="400" liminf="-400"/>
</robot_dk>"""

    with open("test_robot.xml", "w") as f:
        f.write(test_xml_content)

    print("\n--- Test 2: Loading test robot definition ---")
    try:
        sender.load_robot_definition("test_robot.xml")
        loaded_def = sender.get_loaded_robot_definition()
        print(f"Loaded Robot Name: {loaded_def.name}")
        print(f"Loaded Num Joints: {loaded_def.get_num_joints()}")

        if loaded_def.get_num_joints() > 0:
            min_angles, max_angles = loaded_def.get_joint_limits()
            print(f"Joint 1 limits: [{min_angles[0]}, {max_angles[0]}]")

        print("\n--- Test 3: FK Test ---")
        test_angles = [0, 0, 0, 0, 0, 0]
        pos, rpy = sender.calculate_fk(test_angles)
        if pos and rpy:
            print(f"FK Result - Position: {pos}, Orientation (RPY): {rpy}")
        else:
            print("FK failed")

        print("\n--- Test 4: IK Test ---")
        if pos and rpy:
            # Use FK result as IK target
            target_pose = pos + rpy
            ik_solution = sender.calculate_ik(target_pose)
            if ik_solution:
                print(f"IK Solution: {ik_solution}")
                # Verify with FK
                verify_pos, verify_rpy = sender.calculate_fk(ik_solution)
                if verify_pos and verify_rpy:
                    print(
                        f"Verification - Position: {verify_pos}, Orientation: {verify_rpy}"
                    )
                    pos_error = np.linalg.norm(np.array(verify_pos) - np.array(pos))
                    rpy_error = np.linalg.norm(np.array(verify_rpy) - np.array(rpy))
                    print(f"Position error: {pos_error:.6f} mm")
                    print(f"Orientation error: {rpy_error:.6f} degrees")
            else:
                print("IK failed")

    except Exception as e:
        print(f"Error in test: {e}")
        logging.error(f"Test error: {e}", exc_info=True)
