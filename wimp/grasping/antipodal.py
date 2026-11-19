"""Antipodal grasp sampling for a parallel-jaw gripper (WSG 50).

Method (course ch. 5, "antipodal grasps"; docs/DECISIONS.md D-012):
  1. Take surface points with outward normals: from the known phone model
     now, from the camera point cloud later.
  2. For a point p_i with normal n_i, the closing axis is a = -n_i. Find the
     point p_j where the line p_i + t a leaves the object (the farthest
     cloud point within `line_radius` of the line, with n_j . a > 0).
  3. Keep the pair if it is antipodal: n_j . a >= cos(cone_angle), i.e. both
     contact forces along the closing axis lie inside the friction cones.
     Also require the width to fit the opened gripper.
  4. Place the gripper: x_G along a (closing direction), y_G (approach) at
     sampled angles about a, and the contact line at a few positions along
     the finger pads.
  5. Score = antipodal quality + preference for top-down approaches - distance
     of the grasp from the object's centre of mass (less torque when lifting).

Collisions are not checked here. Base placement and IK check every
candidate with the full robot in the scene (wimp/grasping/base_placement.py).
"""

from dataclasses import dataclass
from typing import List

import numpy as np
from pydrake.all import RigidTransform, RotationMatrix

# WSG 50 geometry in its body frame G (measured from the drake_models SDF):
# finger pads span y in [0.098, 0.118]; the pads are 0.107 m apart when open.
PAD_CENTER_Y = 0.108
FINGERTIP_Y = 0.118
MAX_GRASP_WIDTH = 0.09  # leave ~8 mm per side when approaching open

# Scoring weights (D-012).
W_TOP_DOWN = 1.0
W_COM = 5.0  # per metre from the centre of mass


@dataclass
class Grasp:
    X_WG: RigidTransform  # gripper body frame at the grasp
    width: float  # distance between the two contacts
    score: float

    def pregrasp(self, standoff=0.10) -> RigidTransform:
        """Same orientation, backed off along the approach axis (-y_G)."""
        return self.X_WG.multiply(RigidTransform([0.0, -standoff, 0.0]))


def sample_box_surface(X_WO: RigidTransform, size, n: int, rng) -> tuple:
    """Area-weighted random points on a box's faces, with outward normals,
    in world coordinates. Stands in for a segmented point cloud."""
    size = np.asarray(size, float)
    half = size / 2
    faces = []  # (axis, sign, area)
    for ax in range(3):
        area = np.prod(np.delete(size, ax))
        faces += [(ax, 1.0, area), (ax, -1.0, area)]
    areas = np.array([f[2] for f in faces])
    choice = rng.choice(len(faces), size=n, p=areas / areas.sum())
    P = rng.uniform(-half, half, size=(n, 3))
    N = np.zeros((n, 3))
    for k, f in enumerate(choice):
        ax, sign, _ = faces[f]
        P[k, ax] = sign * half[ax]
        N[k, ax] = sign
    R = X_WO.rotation().matrix()
    return (R @ P.T).T + X_WO.translation(), (R @ N.T).T


def antipodal_grasps(
    points: np.ndarray,
    normals: np.ndarray,
    rng,
    num_seeds: int = 300,
    line_radius: float = 0.004,
    cone_angle_deg: float = 15.0,
    num_angles: int = 8,
    tip_shifts=(0.003, 0.006, 0.009),
) -> List[Grasp]:
    """Return grasp candidates sorted best first. `tip_shifts` moves the
    contact line from the pad centre toward the fingertips, so the fingertips
    stick out less past the contacts. A thin object lying on a table needs
    this to keep the fingertips off the table."""
    com = points.mean(axis=0)  # uniform-density approximation
    cos_cone = np.cos(np.deg2rad(cone_angle_deg))
    down = np.array([0.0, 0.0, -1.0])
    out = []
    seeds = rng.choice(len(points), size=min(num_seeds, len(points)), replace=False)
    for i in seeds:
        p_i, a = points[i], -normals[i]
        v = points - p_i
        t = v @ a
        perp = np.linalg.norm(v - np.outer(t, a), axis=1)
        ok = (perp < line_radius) & (t > 0.005) & (normals @ a >= cos_cone)
        if not ok.any():
            continue
        j = np.flatnonzero(ok)[np.argmax(t[ok])]
        width = t[j]
        if width > MAX_GRASP_WIDTH:
            continue
        quality = float(normals[j] @ a)  # 1 = perfectly antipodal
        mid = p_i + 0.5 * width * a

        # Orthonormal basis (u, w) perpendicular to the closing axis.
        helper = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        u = np.cross(a, helper)
        u /= np.linalg.norm(u)
        w = np.cross(a, u)
        for theta in np.linspace(0, 2 * np.pi, num_angles, endpoint=False):
            y = np.cos(theta) * u + np.sin(theta) * w  # approach direction
            z = np.cross(a, y)
            R = RotationMatrix(np.column_stack([a, y, z]))
            top_down = float(y @ down)
            for e in tip_shifts:
                p_WG = mid - (PAD_CENTER_Y + e) * y
                score = quality + W_TOP_DOWN * top_down - W_COM * np.linalg.norm(mid - com)
                out.append(Grasp(RigidTransform(R, p_WG), float(width), score))
    out.sort(key=lambda g: -g.score)
    return out
