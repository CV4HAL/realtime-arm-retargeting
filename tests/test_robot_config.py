"""Tests for safety.robot_config (stage 1: YAML, stage 2: URDF cross-check).

Stage 2 uses a fake URDF object (duck typing), so these tests need neither
yourdfpy nor the mesh files to be parsed.
"""

import dataclasses
import re
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from safety.robot_config import (
    RobotConfigError,
    load_robot_config,
    validate_against_urdf,
)

LITE6_DIR = Path(__file__).resolve().parent.parent / "robots" / "lite6"
LITE6_YAML = LITE6_DIR / "robot.yaml"


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
@pytest.fixture
def robot_dir(tmp_path):
    """A disposable copy of robots/lite6 that each test may modify."""
    dst = tmp_path / "lite6"
    shutil.copytree(LITE6_DIR, dst)
    return dst


def write_config(robot_dir, mutate):
    """Load the real robot.yaml as a dict, apply `mutate(data)`, write it back."""
    path = robot_dir / "robot.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    mutate(data)
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def fake_urdf(links=None, joints=None, actuated=None):
    """Minimal object with the three attributes validate_against_urdf uses."""
    links = links or ["link_base", *(f"link{i}" for i in range(1, 7)), "link_eef"]
    joints = joints or {f"joint{i}": "revolute" for i in range(1, 7)} | {"joint_eef": "fixed"}
    if actuated is None:
        actuated = [name for name, kind in joints.items() if kind != "fixed"]
    return SimpleNamespace(
        link_map={name: object() for name in links},
        joint_map={name: SimpleNamespace(type=kind) for name, kind in joints.items()},
        actuated_joint_names=actuated,
    )


# --------------------------------------------------------------------------- #
# Stage 1: the real file
# --------------------------------------------------------------------------- #
def test_real_lite6_config_loads():
    cfg = load_robot_config(LITE6_YAML)

    assert cfg.name == "ufactory_lite6"
    assert cfg.joint_names == tuple(f"joint{i}" for i in range(1, 7))
    assert all(j.sign == 1 and j.offset_deg == 0.0 for j in cfg.joints)
    assert cfg.tool.parent_link == "link_eef"
    assert cfg.tool.tcp.xyz_m == (0.0, 0.0, 0.0836)
    assert cfg.driver.hw_safety["tool_collision_model"] == 7
    # Paths are resolved to absolute files next to robot.yaml.
    assert cfg.urdf.is_absolute() and cfg.urdf.is_file()
    assert cfg.tool.mesh.is_file()


def test_config_is_immutable():
    cfg = load_robot_config(LITE6_YAML)
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.name = "other"
    with pytest.raises(TypeError):
        cfg.driver.hw_safety["self_collision"] = False


# --------------------------------------------------------------------------- #
# Stage 1: broken files must fail with the key path in the message
# --------------------------------------------------------------------------- #
def _set(path, value):
    """Return a mutator that sets data[path...] = value."""

    def mutate(data):
        node = data
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value

    return mutate


def _delete(key):
    return lambda data: data.pop(key)


BROKEN_CASES = [
    ("sign_not_unit", _set(["joints", 2, "sign"], 2), "joints[2].sign: expected 1 or -1"),
    ("sign_is_bool", _set(["joints", 2, "sign"], True), "joints[2].sign: expected 1 or -1"),
    ("offset_not_number", _set(["joints", 0, "offset_deg"], "zero"),
     "joints[0].offset_deg: expected a number"),
    ("offset_nan", _set(["joints", 0, "offset_deg"], float("nan")),
     "joints[0].offset_deg: expected a finite number"),
    ("duplicate_joint", _set(["joints", 5, "urdf"], "joint5"), "joints[5].urdf: duplicate joint 'joint5'"),
    ("empty_joints", _set(["joints"], []), "joints: expected a non-empty list"),
    ("tcp_two_values", _set(["tool", "tcp", "xyz_m"], [0.0, 0.0836]),
     "tool.tcp.xyz_m: expected 3 numbers, got 2"),
    ("typo_in_key", lambda d: d["tool"].update(tpc=d["tool"].pop("tcp")), "tool: unknown key(s) ['tpc']"),
    ("missing_name", _delete("name"), "name: missing required key"),
    ("missing_mesh_file", _set(["tool", "mesh"], "visual/nope.stl"), "tool.mesh: file not found"),
    ("missing_urdf_file", _set(["description", "urdf"], "nope.urdf"), "description.urdf: file not found"),
]


@pytest.mark.parametrize(
    ("mutate", "message"),
    [case[1:] for case in BROKEN_CASES],
    ids=[case[0] for case in BROKEN_CASES],
)
def test_broken_config_is_rejected(robot_dir, mutate, message):
    path = write_config(robot_dir, mutate)
    with pytest.raises(RobotConfigError, match=re.escape(message)):
        load_robot_config(path)


def test_empty_file_is_rejected(robot_dir):
    path = robot_dir / "robot.yaml"
    path.write_text("", encoding="utf-8")
    with pytest.raises(RobotConfigError, match="file is empty"):
        load_robot_config(path)


def test_missing_file_is_rejected(tmp_path):
    with pytest.raises(RobotConfigError, match="file not found"):
        load_robot_config(tmp_path / "robot.yaml")


def test_acm_may_be_null(robot_dir):
    path = write_config(robot_dir, _set(["acm"], None))
    assert load_robot_config(path).acm is None


# --------------------------------------------------------------------------- #
# Stage 2: cross-check against a (fake) URDF
# --------------------------------------------------------------------------- #
def test_stage2_accepts_matching_urdf():
    validate_against_urdf(load_robot_config(LITE6_YAML), fake_urdf())


def test_stage2_rejects_unknown_tip_link():
    cfg = dataclasses.replace(load_robot_config(LITE6_YAML), tip_link="link_tcp")
    expected = "description.tip_link: link 'link_tcp' not found"
    with pytest.raises(RobotConfigError, match=re.escape(expected)):
        validate_against_urdf(cfg, fake_urdf())


def test_stage2_rejects_fixed_joint():
    joints = {f"joint{i}": "revolute" for i in range(1, 7)} | {"joint_eef": "fixed"}
    joints["joint3"] = "fixed"
    expected = "joints[2].urdf: joint 'joint3' is 'fixed'"
    with pytest.raises(RobotConfigError, match=re.escape(expected)):
        validate_against_urdf(load_robot_config(LITE6_YAML), fake_urdf(joints=joints))


def test_stage2_rejects_unmapped_actuated_joint():
    actuated = [f"joint{i}" for i in range(1, 7)] + ["finger_joint1"]
    with pytest.raises(RobotConfigError, match="not mapped to the driver: finger_joint1"):
        validate_against_urdf(load_robot_config(LITE6_YAML), fake_urdf(actuated=actuated))
