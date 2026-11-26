"""World-frame voxel mapping and collision checks against robot collision geometry.

Occupied voxels are conservatively checked against the robot's registered
Drake collision shapes. This module deliberately has no scene/model loading at
import time; callers provide the plant, scene graph, body poses, and point cloud.
"""

from typing import NamedTuple, Sequence

import numpy as np
from scipy.spatial import cKDTree

from pydrake.all import AbstractValue, LeafSystem, PointCloud, RigidTransform
from pydrake.geometry import Box, Capsule, Cylinder, Sphere

VOXEL_SIZE = 0.03
MIN_HITS = 2
MAP_PERIOD = 0.1
SELF_MASK_MARGIN = 0.05


class RobotCollisionGeometry(NamedTuple):
    """A Drake collision shape attached to a body, with its body-frame pose."""

    body_index: int
    shape: object
    X_BG: RigidTransform


class VoxelMap:
    def __init__(self, voxel_size: float = VOXEL_SIZE):
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        self.s = float(voxel_size)
        self.hits = {}  # (i, j, k) -> number of observations
        self._tree = None
        self._occupied = np.empty((0, 3))
        self._dirty = True

    def integrate(self, pts_W):
        """Integrate Nx3 world-frame points into the persistent hit map."""
        pts_W = np.asarray(pts_W)
        if pts_W.size == 0:
            return
        if pts_W.ndim != 2 or pts_W.shape[1] != 3:
            raise ValueError("pts_W must have shape (N, 3)")
        pts_W = pts_W[np.isfinite(pts_W).all(axis=1)]
        if len(pts_W) == 0:
            return
        idx = np.unique(np.floor(pts_W / self.s).astype(int), axis=0)
        for key in map(tuple, idx):
            self.hits[key] = self.hits.get(key, 0) + 1
        self._dirty = True

    def occupied_centers(self, min_hits: int = MIN_HITS):
        keys = [k for k, n in self.hits.items() if n >= min_hits]
        if not keys:
            return np.empty((0, 3))
        return (np.asarray(keys, dtype=float) + 0.5) * self.s

    def tree(self):
        """KD-tree of occupied voxel centers, cached until the map changes."""
        if self._dirty:
            self._occupied = self.occupied_centers()
            self._tree = cKDTree(self._occupied) if len(self._occupied) else None
            self._dirty = False
        return self._tree

    def _occupied_centers_cached(self):
        self.tree()
        return self._occupied


def robot_collision_geometries(plant, robot_body_indices, scene_graph):
    """Return each robot collision shape and its pose in its body frame."""
    inspector = scene_graph.model_inspector()
    geometries = []
    for body_index in robot_body_indices:
        body = plant.get_body(body_index)
        for geometry_id in plant.GetCollisionGeometriesForBody(body):
            geometries.append(RobotCollisionGeometry(
                int(body_index),
                inspector.GetShape(geometry_id),
                inspector.GetPoseInFrame(geometry_id),
            ))
    return geometries


def robot_geometry_poses(body_poses, geometries: Sequence[RobotCollisionGeometry]):
    """Evaluate world poses for registered robot collision geometries."""
    return [
        (geometry.shape, body_poses[geometry.body_index] @ geometry.X_BG)
        for geometry in geometries
    ]


def _shape_bounding_radius(shape) -> float:
    """Return the radius of a sphere enclosing a supported Drake primitive."""
    if isinstance(shape, Sphere):
        return float(shape.radius())
    if isinstance(shape, Box):
        return float(np.linalg.norm([
            shape.width() / 2, shape.depth() / 2, shape.height() / 2
        ]))
    if isinstance(shape, Cylinder):
        return float(np.hypot(shape.radius(), shape.length() / 2))
    if isinstance(shape, Capsule):
        return float(shape.radius() + shape.length() / 2)
    raise TypeError(
        f"Unsupported robot collision shape {type(shape).__name__}; "
        "add a conservative distance implementation before using it"
    )


def _signed_distance_to_shape(points_G, shape) -> np.ndarray:
    """Signed point-to-shape distance in the shape frame for Drake primitives.

    Negative values are inside. Cylinder and capsule axes are local z, as in
    Drake's geometry convention.
    """
    p = np.asarray(points_G, dtype=float)
    if isinstance(shape, Sphere):
        return np.linalg.norm(p, axis=1) - shape.radius()
    if isinstance(shape, Box):
        q = np.abs(p) - np.array([
            shape.width(), shape.depth(), shape.height()
        ]) / 2
        return (np.linalg.norm(np.maximum(q, 0), axis=1)
                + np.minimum(np.max(q, axis=1), 0))
    if isinstance(shape, Cylinder):
        radial = np.linalg.norm(p[:, :2], axis=1) - shape.radius()
        axial = np.abs(p[:, 2]) - shape.length() / 2
        q = np.column_stack((radial, axial))
        return (np.linalg.norm(np.maximum(q, 0), axis=1)
                + np.minimum(np.max(q, axis=1), 0))
    if isinstance(shape, Capsule):
        z = np.clip(p[:, 2], -shape.length() / 2, shape.length() / 2)
        axis_points = np.column_stack((np.zeros(len(p)), np.zeros(len(p)), z))
        return np.linalg.norm(p - axis_points, axis=1) - shape.radius()
    raise TypeError(
        f"Unsupported robot collision shape {type(shape).__name__}; "
        "add a conservative distance implementation before using it"
    )


def _points_in_geometry_frame(points_W, X_WG):
    # Row-vector form of p_G = R_GW (p_W - p_WG).
    return (np.asarray(points_W) - X_WG.translation()) @ X_WG.rotation().matrix()


def _points_near_geometry(points_W, shape, X_WG, clearance):
    """Select points within clearance of a transformed collision shape."""
    points_W = np.asarray(points_W)
    if len(points_W) == 0:
        return np.zeros(0, dtype=bool)
    broad_radius = _shape_bounding_radius(shape) + clearance
    near = np.linalg.norm(points_W - X_WG.translation(), axis=1) <= broad_radius
    result = np.zeros(len(points_W), dtype=bool)
    if np.any(near):
        points_G = _points_in_geometry_frame(points_W[near], X_WG)
        result[near] = _signed_distance_to_shape(points_G, shape) <= clearance
    return result


def in_collision(voxel_map: VoxelMap, robot_geometry_poses_W,
                 min_hits: int = MIN_HITS) -> bool:
    """Check actual robot collision shapes against observed occupied voxels.

    The center of a voxel is checked with a radius of sqrt(3)*voxel_size/2,
    enclosing the voxel cube. This is conservative and may report a collision
    near a voxel corner. This checks observed occupied space only; it does not
    treat unknown or unobserved space as free-space evidence.
    """
    centers = (voxel_map._occupied_centers_cached() if min_hits == MIN_HITS
               else voxel_map.occupied_centers(min_hits=min_hits))
    if len(centers) == 0:
        return False

    tree = voxel_map.tree() if min_hits == MIN_HITS else cKDTree(centers)
    voxel_radius = np.sqrt(3) * voxel_map.s / 2
    for shape, X_WG in robot_geometry_poses_W:
        candidates = tree.query_ball_point(
            X_WG.translation(), _shape_bounding_radius(shape) + voxel_radius
        )
        if not candidates:
            continue
        points_G = _points_in_geometry_frame(centers[candidates], X_WG)
        if np.any(_signed_distance_to_shape(points_G, shape) <= voxel_radius):
            return True
    return False


def points_inside_robot_geometry(points_W, robot_geometry_poses_W,
                                 margin: float = SELF_MASK_MARGIN):
    """Mask depth returns on or near the robot using its collision geometries."""
    points_W = np.asarray(points_W)
    masked = np.zeros(len(points_W), dtype=bool)
    for shape, X_WG in robot_geometry_poses_W:
        masked |= _points_near_geometry(points_W, shape, X_WG, margin)
    return masked


class VoxelMapper(LeafSystem):
    """Periodically integrate a point cloud after masking robot self-returns."""

    def __init__(self, voxel_map: VoxelMap, robot_geometries,
                 period: float = MAP_PERIOD):
        super().__init__()
        self.map = voxel_map
        self.robot_geometries = list(robot_geometries)
        self.cloud_port = self.DeclareAbstractInputPort(
            "point_cloud", AbstractValue.Make(PointCloud()))
        self.poses_port = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))
        self.DeclarePeriodicPublishEvent(period, 0.0, self._update)

    def _update(self, context):
        cloud = self.cloud_port.Eval(context)
        body_poses = self.poses_port.Eval(context)
        pts_W = np.asarray(cloud.xyzs()).T
        pts_W = pts_W[np.isfinite(pts_W).all(axis=1)]
        if len(pts_W) == 0:
            return
        geometry_poses_W = robot_geometry_poses(body_poses, self.robot_geometries)
        pts_W = pts_W[~points_inside_robot_geometry(pts_W, geometry_poses_W)]
        self.map.integrate(pts_W)
