"""Base navigation planner: RRT-Connect in (x, y, yaw) with the arm stowed.

Collision checking uses the robot's full 3D collision geometry (chassis box,
iiwa primitives, gripper, camera) against the scene, via Drake's SceneGraphCollisionChecker.
Assumes the whole scene is known (privileged information) -- the milestone
this module serves. See docs/DECISIONS.md D-002..D-007.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from pydrake.all import (
    PiecewisePolynomial,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)

from wimp import scene
from wimp.planning.rrt_connect import (
    ConfigSpace,
    RRTConnectResult,
    path_length,
    rrt_connect,
    shortcut,
)

# --- Tunable parameters (each tied to a decision in docs/DECISIONS.md) ------
# D-004: metres of yaw "cost" per radian ~ how far the robot's outer edge
# moves when it spins 1 rad (chassis half-diagonal is 0.5 m).
YAW_WEIGHT = 0.5
# D-003: minimum clearance (m) the robot must keep from the environment.
ENV_PADDING = 0.03
# D-002: RRT-Connect extension step in the weighted metric (m).
RRT_STEP = 0.3
EDGE_RESOLUTION = 0.05
# D-006: velocity limits used to time the path.
V_MAX = 0.5  # m/s
W_MAX = 0.8  # rad/s


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def base_distance(a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    dyaw = wrap_angle(b[2] - a[2])
    return float(np.sqrt(dx * dx + dy * dy + (YAW_WEIGHT * dyaw) ** 2))


def base_interpolate(a, b, t):
    """Straight line in x, y; shortest way round in yaw."""
    out = a + t * (b - a)
    out[2] = a[2] + t * wrap_angle(b[2] - a[2])
    return out


@dataclass
class BasePlan:
    raw_path: List[np.ndarray]
    path: List[np.ndarray]  # after shortcutting, yaw unwrapped
    rrt: RRTConnectResult
    trajectory: PiecewisePolynomial  # (x, y, yaw) over time


class BasePlanner:
    def __init__(self, scene_file: str = scene.SCENE_FILE):
        builder = RobotDiagramBuilder(time_step=0.0)
        scene.load_scene(builder.parser(), scene_file)
        self.diagram = builder.Build()
        self.plant = self.diagram.plant()
        self.idx = scene.robot_indices(self.plant)

        self.checker = SceneGraphCollisionChecker(
            model=self.diagram,
            robot_model_instances=scene.robot_instances(self.plant),
            edge_step_size=EDGE_RESOLUTION,
            env_collision_padding=ENV_PADDING,
            self_collision_padding=0.0,
        )
        # Full-plant configuration used as a template: arm stowed at its
        # default (scene file) posture, phone etc. at their defaults.
        self.q_template = self.plant.GetPositions(self.plant.CreateDefaultContext())

        lo = -scene.ROOM_HALF_EXTENT
        hi = scene.ROOM_HALF_EXTENT
        self.cspace = ConfigSpace(
            sample=lambda rng: np.array(
                [rng.uniform(lo, hi), rng.uniform(lo, hi), rng.uniform(-np.pi, np.pi)]
            ),
            distance=base_distance,
            interpolate=base_interpolate,
            is_free=self.is_free,
            edge_is_free=self.edge_is_free,
            edge_resolution=EDGE_RESOLUTION,
        )

    # --- configuration helpers ---------------------------------------------
    def default_base_q(self) -> np.ndarray:
        return self.q_template[self.idx.base_q].copy()

    def full_q(self, q_base) -> np.ndarray:
        q = self.q_template.copy()
        q[self.idx.base_q] = q_base
        return q

    def is_free(self, q_base) -> bool:
        return bool(self.checker.CheckConfigCollisionFree(self.full_q(q_base)))

    def edge_is_free(self, a, b) -> bool:
        n = max(1, int(np.ceil(base_distance(a, b) / EDGE_RESOLUTION)))
        qs = [self.full_q(base_interpolate(a, b, i / n)) for i in range(1, n + 1)]
        return all(self.checker.CheckConfigsCollisionFree(qs))

    # --- planning -----------------------------------------------------------
    def plan(
        self,
        q_start,
        q_goal,
        seed: Optional[int] = None,
        max_iterations: int = 5000,
        shortcut_iterations: int = 200,
    ) -> Optional[BasePlan]:
        rng = np.random.default_rng(seed)
        q_start = np.asarray(q_start, float)
        q_goal = np.asarray(q_goal, float)
        res = rrt_connect(
            q_start, q_goal, self.cspace, step=RRT_STEP,
            max_iterations=max_iterations, rng=rng,
        )
        if res.path is None:
            return None
        short = shortcut(res.path, self.cspace, shortcut_iterations, rng)
        short = unwrap_yaw(short)
        return BasePlan(
            raw_path=res.path,
            path=short,
            rrt=res,
            trajectory=time_parameterize(short),
        )

    @staticmethod
    def describe(plan: BasePlan) -> str:
        return (
            f"RRT-Connect: {plan.rrt.iterations} iterations, "
            f"{len(plan.rrt.start_tree.nodes) + len(plan.rrt.goal_tree.nodes)} nodes | "
            f"raw path {len(plan.raw_path)} pts, length {path_length(plan.raw_path, base_distance):.2f} | "
            f"shortcut path {len(plan.path)} pts, length {path_length(plan.path, base_distance):.2f} | "
            f"duration {plan.trajectory.end_time():.1f} s"
        )


def unwrap_yaw(path):
    """Make yaw continuous along the path (no +-2pi jumps) so the joint-space
    trajectory turns the short way, matching base_interpolate."""
    out = [np.asarray(q, float).copy() for q in path]
    for prev, cur in zip(out[:-1], out[1:]):
        cur[2] = prev[2] + wrap_angle(cur[2] - prev[2])
    return out


def time_parameterize(path, v_max=V_MAX, w_max=W_MAX) -> PiecewisePolynomial:
    """D-006: stop-at-each-waypoint cubic timing.

    Each straight segment uses s(tau) = 3 tau^2 - 2 tau^3 (zero velocity at
    both ends), so the robot stays exactly on the collision-checked straight
    segments. Peak speed is 1.5 L / T, so T = 1.5 * max(L_xy / v_max,
    |dyaw| / w_max) respects the limits.
    """
    Q = np.array(path).T  # 3 x N
    times = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        L_xy = np.hypot(b[0] - a[0], b[1] - a[1])
        T = 1.5 * max(L_xy / v_max, abs(b[2] - a[2]) / w_max, 0.1)
        times.append(times[-1] + T)
    Qdot = np.zeros_like(Q)
    return PiecewisePolynomial.CubicHermite(np.array(times), Q, Qdot)
