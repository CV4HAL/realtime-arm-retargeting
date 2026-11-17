"""Load and validate a robot description config (robots/<robot>/robot.yaml).

Validation runs in two stages so that typos are caught:

1. ``load_robot_config(path)`` checks the YAML itself: required keys, unknown
   keys (typos), types, ranges and that every referenced file exists. It does
   not parse the URDF, so it is fast and its tests need no robot libraries.
2. ``validate_against_urdf(cfg, urdf)`` cross-checks the names in the config
   against a parsed URDF (a ``yourdfpy.URDF``): links and joints must exist and
   every actuated URDF joint must be mapped to the driver.

Every error is a ``RobotConfigError`` whose message starts with the key path
that failed (e.g. ``tool.tcp.xyz_m: expected 3 numbers, got 2``).

Units are explicit in the key names: ``*_m`` metres, ``*_deg`` degrees.
Relative paths are resolved against the folder that contains the YAML file.

Example::

    cfg = load_robot_config("robots/lite6/robot.yaml")
    urdf = yourdfpy.URDF.load(str(cfg.urdf), load_meshes=False)
    validate_against_urdf(cfg, urdf)
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml

MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})


class RobotConfigError(ValueError):
    """Raised when robot.yaml is missing, malformed or inconsistent with the URDF."""


# --------------------------------------------------------------------------- #
# Data model (immutable: a config must not change while the robot is running)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Pose:
    xyz_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]


@dataclass(frozen=True)
class JointMap:
    """Maps one driver joint (by position in ``RobotConfig.joints``) to the URDF.

    q_urdf_deg = sign * q_driver_deg + offset_deg
    """

    urdf: str
    sign: int
    offset_deg: float


@dataclass(frozen=True)
class ToolConfig:
    name: str
    parent_link: str
    mesh: Path
    origin: Pose  # mesh frame relative to parent_link
    tcp: Pose  # tool centre point relative to parent_link


@dataclass(frozen=True)
class DriverConfig:
    type: str
    hw_safety: Mapping[str, Any]  # read-only view, driver-specific keys


@dataclass(frozen=True)
class RobotConfig:
    name: str
    root: Path  # folder that contains robot.yaml
    urdf: Path
    base_link: str
    tip_link: str
    joints: tuple[JointMap, ...]  # in driver order
    tool: ToolConfig
    driver: DriverConfig
    acm: Path | None
    source: Path | None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.urdf for j in self.joints)


# --------------------------------------------------------------------------- #
# Stage 1: YAML structure
# --------------------------------------------------------------------------- #
_TOP_KEYS = {"name", "source", "description", "joints", "tool", "driver", "acm"}
_DESCRIPTION_KEYS = {"urdf", "base_link", "tip_link"}
_JOINT_KEYS = {"urdf", "sign", "offset_deg"}
_TOOL_KEYS = {"name", "parent_link", "mesh", "origin", "tcp"}
_POSE_KEYS = {"xyz_m", "rpy_deg"}
_DRIVER_KEYS = {"type", "hw_safety"}


def load_robot_config(path: str | Path) -> RobotConfig:
    """Read robot.yaml, validate its structure and return an immutable config."""
    path = Path(path)
    if not path.is_file():
        raise RobotConfigError(f"{path}: file not found")

    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise RobotConfigError(f"{path}: invalid YAML: {exc}") from exc

    if data is None:
        raise RobotConfigError(f"{path}: file is empty")
    root = path.resolve().parent

    top = _as_mapping(data, "<root>")
    _reject_unknown(top, _TOP_KEYS, "<root>")

    desc = _as_mapping(_require(top, "description", "<root>"), "description")
    _reject_unknown(desc, _DESCRIPTION_KEYS, "description")

    source = top.get("source")
    acm = top.get("acm")

    return RobotConfig(
        name=_as_str(_require(top, "name", "<root>"), "name"),
        root=root,
        urdf=_as_existing_file(_require(desc, "urdf", "description"), root, "description.urdf"),
        base_link=_as_str(_require(desc, "base_link", "description"), "description.base_link"),
        tip_link=_as_str(_require(desc, "tip_link", "description"), "description.tip_link"),
        joints=_parse_joints(_require(top, "joints", "<root>")),
        tool=_parse_tool(_require(top, "tool", "<root>"), root),
        driver=_parse_driver(_require(top, "driver", "<root>")),
        # acm may be null until it is generated; emptiness is the ACM loader's job.
        acm=None if acm is None else _as_existing_file(acm, root, "acm"),
        source=None if source is None else _as_existing_file(source, root, "source"),
    )


def _parse_joints(value: Any) -> tuple[JointMap, ...]:
    if not isinstance(value, list) or not value:
        raise RobotConfigError("joints: expected a non-empty list")

    joints: list[JointMap] = []
    seen: set[str] = set()
    for i, item in enumerate(value):
        where = f"joints[{i}]"
        entry = _as_mapping(item, where)
        _reject_unknown(entry, _JOINT_KEYS, where)

        name = _as_str(_require(entry, "urdf", where), f"{where}.urdf")
        if name in seen:
            raise RobotConfigError(f"{where}.urdf: duplicate joint '{name}'")
        seen.add(name)

        sign = _require(entry, "sign", where)
        # bool is a subclass of int in Python: reject `sign: true` explicitly.
        if isinstance(sign, bool) or sign not in (1, -1):
            raise RobotConfigError(f"{where}.sign: expected 1 or -1, got {sign!r}")

        offset = _as_number(_require(entry, "offset_deg", where), f"{where}.offset_deg")
        joints.append(JointMap(urdf=name, sign=int(sign), offset_deg=offset))
    return tuple(joints)


def _parse_tool(value: Any, root: Path) -> ToolConfig:
    tool = _as_mapping(value, "tool")
    _reject_unknown(tool, _TOOL_KEYS, "tool")
    return ToolConfig(
        name=_as_str(_require(tool, "name", "tool"), "tool.name"),
        parent_link=_as_str(_require(tool, "parent_link", "tool"), "tool.parent_link"),
        mesh=_as_existing_file(_require(tool, "mesh", "tool"), root, "tool.mesh"),
        origin=_as_pose(_require(tool, "origin", "tool"), "tool.origin"),
        tcp=_as_pose(_require(tool, "tcp", "tool"), "tool.tcp"),
    )


def _parse_driver(value: Any) -> DriverConfig:
    driver = _as_mapping(value, "driver")
    _reject_unknown(driver, _DRIVER_KEYS, "driver")
    hw_safety = _as_mapping(driver.get("hw_safety", {}), "driver.hw_safety")
    return DriverConfig(
        type=_as_str(_require(driver, "type", "driver"), "driver.type"),
        hw_safety=MappingProxyType(dict(hw_safety)),
    )


# --------------------------------------------------------------------------- #
# Stage 2: cross-check against the URDF
# --------------------------------------------------------------------------- #
def validate_against_urdf(cfg: RobotConfig, urdf: Any) -> None:
    """Check that every name in ``cfg`` exists in ``urdf`` (a ``yourdfpy.URDF``).

    Only ``urdf.link_map``, ``urdf.joint_map`` and ``urdf.actuated_joint_names``
    are used, so this module does not import yourdfpy itself.
    """
    links = urdf.link_map
    for where, link in (
        ("description.base_link", cfg.base_link),
        ("description.tip_link", cfg.tip_link),
        ("tool.parent_link", cfg.tool.parent_link),
    ):
        if link not in links:
            raise RobotConfigError(f"{where}: link '{link}' not found in {cfg.urdf.name}")

    for i, joint in enumerate(cfg.joints):
        urdf_joint = urdf.joint_map.get(joint.urdf)
        if urdf_joint is None:
            raise RobotConfigError(
                f"joints[{i}].urdf: joint '{joint.urdf}' not found in {cfg.urdf.name}"
            )
        if urdf_joint.type not in MOVABLE_JOINT_TYPES:
            raise RobotConfigError(
                f"joints[{i}].urdf: joint '{joint.urdf}' is '{urdf_joint.type}', not movable"
            )

    # An actuated joint the driver does not report would leave the model with an
    # unknown value, so the collision check would be wrong. Fail instead.
    unmapped = [n for n in urdf.actuated_joint_names if n not in cfg.joint_names]
    if unmapped:
        raise RobotConfigError(
            f"joints: actuated URDF joints not mapped to the driver: {', '.join(unmapped)}"
        )


# --------------------------------------------------------------------------- #
# Small typed readers: each one reports the key path on failure
# --------------------------------------------------------------------------- #
def _require(mapping: Mapping[str, Any], key: str, where: str) -> Any:
    if key not in mapping:
        prefix = "" if where == "<root>" else f"{where}."
        raise RobotConfigError(f"{prefix}{key}: missing required key")
    return mapping[key]


def _reject_unknown(mapping: Mapping[str, Any], allowed: set[str], where: str) -> None:
    unknown = sorted(set(mapping) - allowed)
    if unknown:
        raise RobotConfigError(
            f"{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}"
        )


def _as_mapping(value: Any, where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise RobotConfigError(f"{where}: expected a mapping, got {type(value).__name__}")
    return value


def _as_str(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RobotConfigError(f"{where}: expected a non-empty string, got {value!r}")
    return value


def _as_number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RobotConfigError(f"{where}: expected a number, got {value!r}")
    if not math.isfinite(value):
        raise RobotConfigError(f"{where}: expected a finite number, got {value!r}")
    return float(value)


def _as_vec3(value: Any, where: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        n = len(value) if isinstance(value, (list, tuple)) else type(value).__name__
        raise RobotConfigError(f"{where}: expected 3 numbers, got {n}")
    x, y, z = (_as_number(v, f"{where}[{i}]") for i, v in enumerate(value))
    return (x, y, z)


def _as_pose(value: Any, where: str) -> Pose:
    pose = _as_mapping(value, where)
    _reject_unknown(pose, _POSE_KEYS, where)
    return Pose(
        xyz_m=_as_vec3(_require(pose, "xyz_m", where), f"{where}.xyz_m"),
        rpy_deg=_as_vec3(_require(pose, "rpy_deg", where), f"{where}.rpy_deg"),
    )


def _as_existing_file(value: Any, root: Path, where: str) -> Path:
    rel = _as_str(value, where)
    path = (root / rel).resolve()
    if not path.is_file():
        raise RobotConfigError(f"{where}: file not found: {rel} (resolved to {path})")
    return path
