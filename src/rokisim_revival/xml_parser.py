import logging
import re
from typing import Dict, List, Tuple

from lxml import etree


class XMLRobotParser:
    """Parses robot definition from XML files."""

    def parse(
        self, xml_file_path: str
    ) -> Tuple[str, List[float], List[float], List[Dict[str, float]], Dict[str, int]]:
        """Parse XML and return robot name, min/max angles, DH params, axis index."""
        parser = etree.XMLParser(recover=True, no_network=True, resolve_entities=False)
        with open(xml_file_path, "r", encoding="utf-8") as f:
            content = f.read()
        pattern = r"(<robot_dk\s+[^>]*?name=)([^\s>\"\']+)([\s>])"
        fixed_content = re.sub(pattern, r'\1"\2"\3', content, flags=re.DOTALL)
        root = etree.fromstring(fixed_content.encode("utf-8"), parser)

        robot_dk = root if root.tag == "robot_dk" else root.find("robot_dk")
        if robot_dk is None:
            logging.error(f"XML root children: {[child.tag for child in root]}")
            raise ValueError("No 'robot_dk' element found.")

        name = robot_dk.get("name", "Unknown")
        joint_min_angles: List[float] = []
        joint_max_angles: List[float] = []
        dh_params: List[Dict[str, float]] = []
        axis_id_to_index: Dict[str, int] = {}

        axis_elements = robot_dk.findall("axis")
        for axis in axis_elements:
            axis_id = axis.get("id")
            if not axis_id:
                continue
            try:
                dh_entry = {
                    "alpha": float(axis.get("alpha", 0.0)),
                    "a": float(axis.get("a", 0.0)),
                    "theta": float(axis.get("theta", 0.0)),
                    "d": float(axis.get("d", 0.0)),
                }
                dh_params.append(dh_entry)
                dh_index = len(dh_params) - 1
                axis_id_to_index[axis_id] = dh_index

                limsup = axis.get("limsup")
                liminf = axis.get("liminf")
                maxvalue = axis.get("maxvalue")
                minvalue = axis.get("minvalue")

                has_limits = False
                max_limit = 180.0
                min_limit = -180.0

                if limsup is not None and liminf is not None:
                    max_limit = float(limsup)
                    min_limit = float(liminf)
                    has_limits = True
                elif maxvalue is not None and minvalue is not None:
                    max_limit = float(maxvalue)
                    min_limit = float(minvalue)
                    has_limits = True

                if axis_id.startswith("Joint") and has_limits:
                    joint_min_angles.append(min_limit)
                    joint_max_angles.append(max_limit)
            except (ValueError, TypeError) as e:
                logging.warning(f"Error parsing axis {axis_id}: {e}")

        return name, joint_min_angles, joint_max_angles, dh_params, axis_id_to_index
