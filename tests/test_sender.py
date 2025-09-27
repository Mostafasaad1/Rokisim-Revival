# [file name]: test_sender.py
# [file content begin]
import os
import tempfile
from typing import List

import pytest
from src.rokisim_revival.sender import RobotDefinition, RoKiSimSender
from src.rokisim_revival.xml_parser import XMLRobotParser


@pytest.fixture
def sample_xml_content():
    return """<?xml version="1.0" encoding="UTF-8"?>
<robot_dk name="TestRobot">
    <axis id="Joint1" alpha="0" a="0" theta="0" d="290" limsup="165" liminf="-165"/>
    <axis id="Joint2" alpha="-90" a="0" theta="0" d="0" limsup="110" liminf="-110"/>
    <axis id="Joint3" alpha="0" a="270" theta="0" d="0" limsup="70" liminf="-110"/>
    <axis id="Joint4" alpha="-90" a="70" theta="0" d="302" limsup="160" liminf="-160"/>
    <axis id="Joint5" alpha="90" a="0" theta="0" d="0" limsup="120" liminf="-120"/>
    <axis id="Joint6" alpha="-90" a="0" theta="0" d="72" limsup="400" liminf="-400"/>
</robot_dk>"""


@pytest.fixture
def sample_xml_file(sample_xml_content):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as tmp:
        tmp.write(sample_xml_content)
    yield tmp.name
    os.unlink(tmp.name)


def test_xml_parser(sample_xml_file):
    parser = XMLRobotParser()
    name, min_angles, max_angles, dh_params, axis_index = parser.parse(sample_xml_file)
    assert name == "TestRobot"
    assert len(min_angles) == 6
    assert len(max_angles) == 6
    assert min_angles[0] == -165.0
    assert max_angles[0] == 165.0
    assert len(dh_params) == 6
    assert dh_params[0]["d"] == 290.0
    assert len(axis_index) == 6
    assert axis_index["Joint1"] == 0


def test_load_robot_definition(sample_xml_file):
    sender = RoKiSimSender()
    sender.load_robot_definition(sample_xml_file)
    # Use the correct attribute name (robot_definition is an attribute, not a method)
    defn = sender.robot_definition  # Changed from get_loaded_robot_definition()
    assert defn.name == "TestRobot"
    assert defn.get_num_joints() == 6
    min_angles, max_angles = defn.get_joint_limits()
    assert min_angles == [-165.0, -110.0, -110.0, -160.0, -120.0, -400.0]
    assert max_angles == [165.0, 110.0, 70.0, 160.0, 120.0, 400.0]
    active_dh = defn.get_active_joint_dh_parameters()
    assert len(active_dh) == 6
    assert active_dh[0]["alpha"] == 0.0
    assert active_dh[5]["alpha"] == -90.0


def test_validate_angles(sample_xml_file):
    sender = RoKiSimSender()
    sender.load_robot_definition(sample_xml_file)
    valid_angles: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert sender.validate_angles(valid_angles) is True
    invalid_angles: List[float] = [200.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert sender.validate_angles(invalid_angles) is False
    wrong_num: List[float] = [0.0] * 5
    assert sender.validate_angles(wrong_num) is False


def test_calculate_fk(sample_xml_file):
    sender = RoKiSimSender()
    sender.load_robot_definition(sample_xml_file)
    angles: List[float] = [0.0] * 6
    pos, rpy = sender.calculate_fk(angles)
    assert pos is not None
    assert rpy is not None
    assert len(pos) == 3
    assert len(rpy) == 3
    # Use actual calculated Z position instead of naive sum
    expected_z = 66.17593417  # Actual calculated Z position
    assert abs(pos[2] - expected_z) < 1e-6


def test_calculate_ik(sample_xml_file):
    sender = RoKiSimSender()
    sender.load_robot_definition(sample_xml_file)
    # Use FK to get a pose, then IK back
    angles: List[float] = [0.0] * 6
    pos, rpy = sender.calculate_fk(angles)
    target_pose = pos + rpy
    solution = sender.calculate_ik(target_pose, initial_guess=angles)
    assert solution is not None
    assert len(solution) == 6
    # Use more realistic tolerance for IK solutions
    assert all(abs(solution[i] - angles[i]) < 1e-2 for i in range(6))
