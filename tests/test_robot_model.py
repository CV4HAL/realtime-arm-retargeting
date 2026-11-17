"""Tests for safety.robot_model (forward kinematics from robot.yaml + URDF).

Reference values were computed from robots/lite6/lite6.urdf; they do not need
a robot or the simulator. The comparison against the real controller lives in
test_fk_simulator.py.
"""

import dataclasses
from pathlib import Path

import numpy as np
import pytest

from safety.robot_config import load_robot_config
from safety.robot_model import TOOL_LINK, RobotModel

LITE6_YAML = Path(__file__).resolve().parent.parent / "robots" / "lite6" / "robot.yaml"
Q_ZERO = np.zeros(6)
TOL_M = 1e-4  # 0.1 mm


@pytest.fixture(scope="module")
def cfg():
    return load_robot_config(LITE6_YAML)


@pytest.fixture(scope="module")
def model(cfg):
    return RobotModel(cfg)


def random_configs(n, seed=0):
    """Joint angles (deg, driver order) inside the Lite 6 joint limits."""
    rng = np.random.default_rng(seed)
    low = np.array([-170, -140, 0, -170, -120, -170])
    high = np.array([170, 140, 290, 170, 120, 170])
    return rng.uniform(low, high, size=(n, 6))


# --------------------------------------------------------------------------- #
# Known poses at q = 0
# --------------------------------------------------------------------------- #
def test_flange_at_zero(model):
    flange = model.flange_pose(Q_ZERO)
    np.testing.assert_allclose(flange[:3, 3], [0.087, 0.0, 0.15359], atol=TOL_M)
    # The flange z axis points straight down at q = 0. The URDF rounds pi/2 to
    # 1.5708 and pi to 3.1416, which tilts it by ~7e-6 rad (2 um at 0.3 m): hence 1e-4.
    np.testing.assert_allclose(flange[:3, 2], [0.0, 0.0, -1.0], atol=1e-4)


def test_tcp_at_zero(model):
    np.testing.assert_allclose(model.tcp_pose(Q_ZERO)[:3, 3], [0.087, 0.0, 0.06999], atol=TOL_M)


def test_tool_mesh_spans_flange_to_tcp_at_zero(model):
    """The gripper must hang from the flange (z = 153.59 mm) down to the TCP (z = 69.99 mm)."""
    tool = model.collision_meshes()[TOOL_LINK][0].copy()
    tool.apply_transform(model.link_poses(Q_ZERO)[TOOL_LINK])
    np.testing.assert_allclose(tool.bounds[:, 2], [0.06999, 0.15359], atol=TOL_M)


# --------------------------------------------------------------------------- #
# Invariants for any configuration
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("q", random_configs(20))
def test_tcp_is_fixed_offset_from_flange(model, cfg, q):
    flange_to_tcp = np.linalg.inv(model.flange_pose(q)) @ model.tcp_pose(q)
    np.testing.assert_allclose(flange_to_tcp[:3, 3], cfg.tool.tcp.xyz_m, atol=1e-9)
    np.testing.assert_allclose(flange_to_tcp[:3, :3], np.eye(3), atol=1e-9)


@pytest.mark.parametrize("q", random_configs(5, seed=1))
def test_poses_are_rigid_transforms(model, q):
    for name, pose in model.link_poses(q).items():
        rot = pose[:3, :3]
        np.testing.assert_allclose(rot @ rot.T, np.eye(3), atol=1e-9, err_msg=name)
        assert np.isclose(np.linalg.det(rot), 1.0), name
        np.testing.assert_array_equal(pose[3], [0, 0, 0, 1])


def test_link_poses_cover_every_link_and_the_tool(model):
    poses = model.link_poses(Q_ZERO)
    assert set(poses) == set(model.link_names)
    assert TOOL_LINK in poses and "link6" in poses


# --------------------------------------------------------------------------- #
# Joint mapping (driver -> URDF)
# --------------------------------------------------------------------------- #
def test_driver_to_urdf_identity_mapping(model):
    q = np.array([10.0, -20.0, 30.0, -40.0, 50.0, -60.0])
    np.testing.assert_allclose(model.driver_to_urdf(q), np.radians(q))


def test_driver_to_urdf_applies_sign_and_offset(cfg):
    joints = list(cfg.joints)
    joints[0] = dataclasses.replace(joints[0], sign=-1, offset_deg=10.0)
    model = RobotModel(dataclasses.replace(cfg, joints=tuple(joints)))

    q_urdf = model.driver_to_urdf([30.0, 0, 0, 0, 0, 0])
    assert q_urdf[0] == pytest.approx(np.radians(-30.0 + 10.0))


def test_sign_flip_mirrors_the_pose(model, cfg):
    joints = list(cfg.joints)
    joints[0] = dataclasses.replace(joints[0], sign=-1)
    flipped = RobotModel(dataclasses.replace(cfg, joints=tuple(joints)))

    q = np.array([40.0, -30.0, 60.0, 20.0, -45.0, 10.0])
    q_neg = q.copy()
    q_neg[0] = -q[0]
    np.testing.assert_allclose(flipped.tcp_pose(q), model.tcp_pose(q_neg), atol=1e-12)


@pytest.mark.parametrize("bad_q", [np.zeros(5), np.zeros(7), [0, 0, np.nan, 0, 0, 0]])
def test_invalid_joint_vector_is_rejected(model, bad_q):
    with pytest.raises(ValueError):
        model.tcp_pose(bad_q)


# --------------------------------------------------------------------------- #
# Collision geometry
# --------------------------------------------------------------------------- #
def test_collision_meshes_exist_for_every_body(model):
    meshes = model.collision_meshes()
    expected = {"link_base", "link1", "link2", "link3", "link4", "link5", "link6", TOOL_LINK}
    assert set(meshes) == expected
    for name, items in meshes.items():
        assert items and all(len(m.vertices) > 0 for m in items), name


def test_collision_meshes_have_plausible_size(model):
    """Catches unit errors: a mesh read in mm instead of m would be 1000x larger."""
    for name, items in model.collision_meshes().items():
        for mesh in items:
            assert 0.01 < max(mesh.extents) < 0.5, f"{name}: extents {mesh.extents}"


def test_collision_meshes_are_read_only_view(model):
    with pytest.raises(TypeError):
        model.collision_meshes()["extra"] = ()
