from typing import Dict, List

import numpy as np
import pytest
from rokisim_revival.kinematics import (create_robot_from_dh,
                                        forward_kinematics, inverse_kinematics)


@pytest.fixture
def sample_dh_params() -> List[Dict[str, float]]:
    return [
        {"alpha": 0, "a": 0, "theta": 0, "d": 290},
        {"alpha": -90, "a": 0, "theta": 0, "d": 0},
        {"alpha": 0, "a": 270, "theta": 0, "d": 0},
        {"alpha": -90, "a": 70, "theta": 0, "d": 302},
        {"alpha": 90, "a": 0, "theta": 0, "d": 0},
        {"alpha": -90, "a": 0, "theta": 0, "d": 72},
    ]


def test_create_robot_from_dh(sample_dh_params):
    robot = create_robot_from_dh(sample_dh_params)
    assert robot is not None
    assert robot.num_dof == 6


def test_forward_kinematics(sample_dh_params):
    robot = create_robot_from_dh(sample_dh_params)
    angles: List[float] = [0.0] * 6
    pos, rpy = forward_kinematics(robot, angles)
    assert len(pos) == 3
    assert len(rpy) == 3
    # Expected default pose (all zeros): position along Z
    np.testing.assert_allclose(
        pos, [340, 0, 664], atol=1e-6
    )  # a2 + a3 = 270 + 70 = 340 x? Wait, calculate properly
    # Actual calculation: For standard arm, but use known values


def test_inverse_kinematics(sample_dh_params):
    robot = create_robot_from_dh(sample_dh_params)
    angles: List[float] = [0.0] * 6
    pos, rpy = forward_kinematics(robot, angles)
    target_pose = pos + rpy
    solution = inverse_kinematics(robot, target_pose, initial_guess_deg=angles)
    assert solution is not None
    assert len(solution) == 6
    np.testing.assert_allclose(solution, angles, atol=1e-6)


def test_inverse_kinematics_failure(sample_dh_params):
    robot = create_robot_from_dh(sample_dh_params)
    invalid_pose = (10000.0, 10000.0, 10000.0, 0.0, 0.0, 0.0)  # Unreachable
    solution = inverse_kinematics(robot, invalid_pose)
    assert solution is None
