"""Compare RobotModel against the controller's own forward kinematics.

This is the test that confirms `sign` and `offset_deg` in robot.yaml: if the
URDF model and the controller disagree, the collision checker would be looking
at a robot that does not exist.

It needs the UFACTORY Lite 6 simulator (or a real arm) and is skipped unless
XARM_SIM_IP is set, e.g.:

    XARM_SIM_IP=127.0.0.1 pytest tests/test_fk_simulator.py -v        (bash)
    $env:XARM_SIM_IP="127.0.0.1"; pytest tests/test_fk_simulator.py -v (PowerShell)

get_forward_kinematics() is a pure computation in the controller: the arm does
not move during this test.

Precondition: the controller TCP offset must be zero, so that the controller
returns the flange pose and we compare flange against flange.
"""

import os
from pathlib import Path

import numpy as np
import pytest
import trimesh

from safety.robot_config import load_robot_config
from safety.robot_model import RobotModel

SIM_IP = os.environ.get("XARM_SIM_IP")
pytestmark = pytest.mark.skipif(not SIM_IP, reason="set XARM_SIM_IP to run against the simulator")

LITE6_YAML = Path(__file__).resolve().parent.parent / "robots" / "lite6" / "robot.yaml"

# lite6.urdf is aligned to the manual's DH table, which is what the controller
# uses (see robots/lite6/SOURCE.md). Model and controller must agree almost
# exactly; these tolerances only absorb float rounding in the SDK.
# Before the alignment the error was up to 1.35 mm (1.41 mm analytic bound).
POS_TOL_MM = 0.05
ROT_TOL_DEG = 0.01
N_CONFIGS = 20


@pytest.fixture(scope="module")
def model():
    return RobotModel(load_robot_config(LITE6_YAML))


@pytest.fixture(scope="module")
def arm():
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(SIM_IP, is_radian=False)
    yield arm
    arm.disconnect()


def controller_pose_to_matrix(pose):
    """SDK pose [x, y, z (mm), roll, pitch, yaw (deg)] -> 4x4 matrix in metres.

    The manual defines roll/pitch/yaw as fixed-axis X-Y-Z angles ("sxyz").
    """
    roll, pitch, yaw = np.radians(pose[3:6])
    matrix = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    matrix[:3, 3] = np.asarray(pose[:3]) / 1000.0
    return matrix


def rotation_error_deg(r_a, r_b):
    cos_angle = (np.trace(r_a.T @ r_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))))


def random_configs(n, seed=0):
    rng = np.random.default_rng(seed)
    low = np.array([-170, -140, 0, -170, -120, -170])
    high = np.array([170, 140, 290, 170, 120, 170])
    return rng.uniform(low, high, size=(n, 6))


def test_controller_tcp_offset_is_zero(arm):
    offset = np.asarray(arm.tcp_offset, dtype=float)
    if not np.allclose(offset, 0.0):
        pytest.skip(f"controller TCP offset is {offset.tolist()}; set it to zero to compare flanges")


def _config_id(q):
    return "q=" + ",".join(f"{a:.0f}" for a in q)


@pytest.mark.parametrize("q", random_configs(N_CONFIGS), ids=_config_id)
def test_flange_matches_controller(model, arm, q):
    if not np.allclose(np.asarray(arm.tcp_offset, dtype=float), 0.0):
        pytest.skip("controller TCP offset is not zero")

    code, pose = arm.get_forward_kinematics(list(q), input_is_radian=False, return_is_radian=False)
    assert code == 0, f"get_forward_kinematics failed with code {code}"

    expected = controller_pose_to_matrix(pose)
    ours = model.flange_pose(q)

    pos_err_mm = 1000.0 * np.linalg.norm(ours[:3, 3] - expected[:3, 3])
    rot_err_deg = rotation_error_deg(ours[:3, :3], expected[:3, :3])
    print(f"\n  pos_err_mm={pos_err_mm:.3f}  rot_err_deg={rot_err_deg:.4f}")
    assert pos_err_mm < POS_TOL_MM, f"position error {pos_err_mm:.2f} mm (controller {pose})"
    assert rot_err_deg < ROT_TOL_DEG, f"rotation error {rot_err_deg:.3f} deg (controller {pose})"
