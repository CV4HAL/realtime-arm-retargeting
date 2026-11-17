"""Forward kinematics of the robot and its tool, built from RobotConfig + URDF.

``RobotModel`` answers one question: given the joint angles reported by the
driver, where is every link, the tool and the TCP? It is the input of the
collision checker.

Conventions
-----------
* Input angles are in **driver order and degrees** (what the SDK reports).
  They are mapped to the URDF with ``q_urdf_deg = sign * q_driver_deg + offset_deg``
  (see ``JointMap``) and converted to radians internally.
* All poses are 4x4 homogeneous matrices in **metres**, expressed in
  ``cfg.base_link``.
* Every mesh returned by ``collision_meshes()`` is expressed in the frame that
  ``link_poses()`` returns for the same key, so ``mesh.apply_transform(pose)``
  places it in the base frame.
* The tool is exposed as an extra link named ``TOOL_LINK`` ("tool").

Not thread-safe: ``yourdfpy`` keeps the last configuration as internal state.
Create one ``RobotModel`` per thread (in practice, inside the servo thread).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from types import MappingProxyType

import numpy as np
import trimesh
import yourdfpy

from safety.robot_config import Pose, RobotConfig, validate_against_urdf

TOOL_LINK = "tool"


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Pose (meters, URDF roll-pitch-yaw in degrees) -> 4x4 matrix.

    URDF rpy are fixed-axis X-Y-Z rotations, i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll),
    which is the "sxyz" convention of trimesh.transformations.
    """
    roll, pitch, yaw = np.radians(pose.rpy_deg)
    matrix = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    matrix[:3, 3] = pose.xyz_m
    return matrix


class RobotModel:
    def __init__(self, cfg: RobotConfig) -> None:
        self.cfg = cfg
        self._urdf = yourdfpy.URDF.load(
            str(cfg.urdf),
            mesh_dir=str(cfg.urdf.parent),
            load_meshes=False,  # visual meshes are not needed
            build_scene_graph=True,  # kinematic tree used by get_transform()
            build_collision_scene_graph=True,
            load_collision_meshes=True,
        )
        # Stage 2 of the config validation: names in robot.yaml must exist in the URDF.
        validate_against_urdf(cfg, self._urdf)

        # driver order -> URDF actuated order (they may differ).
        urdf_order = list(self._urdf.actuated_joint_names)
        driver_index = {joint.urdf: i for i, joint in enumerate(cfg.joints)}
        self._driver_to_urdf_idx = np.array([driver_index[name] for name in urdf_order])
        self._sign = np.array([cfg.joints[i].sign for i in self._driver_to_urdf_idx], dtype=float)
        self._offset_deg = np.array(
            [cfg.joints[i].offset_deg for i in self._driver_to_urdf_idx], dtype=float
        )

        self._t_parent_tool = pose_to_matrix(cfg.tool.origin)
        self._t_parent_tcp = pose_to_matrix(cfg.tool.tcp)
        self._link_names = tuple(self._urdf.link_map)
        self._collision_meshes = self._build_collision_meshes()

    # ------------------------------------------------------------------ #
    # Joint mapping
    # ------------------------------------------------------------------ #
    @property
    def link_names(self) -> tuple[str, ...]:
        """URDF links plus the tool, i.e. the keys of ``link_poses()``."""
        return (*self._link_names, TOOL_LINK)

    def driver_to_urdf(self, q_driver_deg: Sequence[float]) -> np.ndarray:
        """Driver angles (deg, driver order) -> URDF actuated angles (rad, URDF order)."""
        q = np.asarray(q_driver_deg, dtype=float)
        if q.shape != (len(self.cfg.joints),):
            raise ValueError(
                f"expected {len(self.cfg.joints)} joint angles in driver order, got shape {q.shape}"
            )
        if not np.all(np.isfinite(q)):
            raise ValueError(f"joint angles must be finite, got {q.tolist()}")
        q_urdf_deg = self._sign * q[self._driver_to_urdf_idx] + self._offset_deg
        return np.radians(q_urdf_deg)

    # ------------------------------------------------------------------ #
    # Forward kinematics
    # ------------------------------------------------------------------ #
    def link_poses(self, q_driver_deg: Sequence[float]) -> dict[str, np.ndarray]:
        """Pose of every URDF link and of the tool, in ``base_link`` (metres)."""
        self._update(q_driver_deg)
        poses = {name: self._base_to(name) for name in self._link_names}
        poses[TOOL_LINK] = poses[self.cfg.tool.parent_link] @ self._t_parent_tool
        return poses

    def tool_pose(self, q_driver_deg: Sequence[float]) -> np.ndarray:
        """Pose of the tool mesh frame in ``base_link``."""
        self._update(q_driver_deg)
        return self._base_to(self.cfg.tool.parent_link) @ self._t_parent_tool

    def tcp_pose(self, q_driver_deg: Sequence[float]) -> np.ndarray:
        """Pose of the tool centre point in ``base_link``."""
        self._update(q_driver_deg)
        return self._base_to(self.cfg.tool.parent_link) @ self._t_parent_tcp

    def flange_pose(self, q_driver_deg: Sequence[float]) -> np.ndarray:
        """Pose of ``tip_link`` (the flange) in ``base_link``."""
        self._update(q_driver_deg)
        return self._base_to(self.cfg.tip_link)

    # ------------------------------------------------------------------ #
    # Geometry
    # ------------------------------------------------------------------ #
    def collision_meshes(self) -> Mapping[str, tuple[trimesh.Trimesh, ...]]:
        """Collision meshes per link (and the tool), each in its own link frame.

        Read-only view built once at start-up. Do not modify the meshes:
        copy them (``mesh.copy()``) before transforming.
        """
        return self._collision_meshes

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _update(self, q_driver_deg: Sequence[float]) -> None:
        self._urdf.update_cfg(self.driver_to_urdf(q_driver_deg))

    def _base_to(self, frame: str) -> np.ndarray:
        return np.array(self._urdf.get_transform(frame, self.cfg.base_link), dtype=float)

    def _build_collision_meshes(self) -> Mapping[str, tuple[trimesh.Trimesh, ...]]:
        """Express every URDF collision geometry in the frame of the link that owns it.

        The transform is computed as inv(T_base_link) @ T_base_geometry at the zero
        configuration, so it is correct however yourdfpy stores the <collision>
        <origin> (baked into vertices or as a scene-graph edge).
        """
        self._update(np.zeros(len(self.cfg.joints)))
        scene = self._urdf.collision_scene
        parents = scene.graph.transforms.parents
        meshes: dict[str, list[trimesh.Trimesh]] = {}
        for node in scene.graph.nodes_geometry:
            # Walk up the scene graph until reaching the link that owns this geometry.
            link = node
            while link is not None and link not in self._urdf.link_map:
                link = parents.get(link)
            if link is None:
                raise RuntimeError(f"collision geometry '{node}' is not attached to any link")
            t_base_geom, geom_name = scene.graph.get(node)
            t_link_geom = np.linalg.inv(self._base_to(link)) @ t_base_geom
            mesh = scene.geometry[geom_name].copy()
            mesh.apply_transform(t_link_geom)
            meshes.setdefault(link, []).append(mesh)

        missing = [
            link.name for link in self._urdf.robot.links if link.collisions and link.name not in meshes
        ]
        if missing:
            raise RuntimeError(f"collision geometry could not be loaded for links: {missing}")

        tool_mesh = trimesh.load(str(self.cfg.tool.mesh), force="mesh")
        meshes[TOOL_LINK] = [tool_mesh]  # already in the tool frame (tool.origin is its pose)
        return MappingProxyType({name: tuple(items) for name, items in meshes.items()})
