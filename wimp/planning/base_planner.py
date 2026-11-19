"""Base navigation planner: RRT-Connect in (x, y, yaw) with the arm stowed,
using turn-drive-turn edges for the skid-steer base (D-015).

Collision checking uses the robot's full 3D collision geometry (chassis box,
iiwa primitives, gripper, camera) against the scene via the shared
PlanningScene. See docs/DECISIONS.md D-002..D-006.
"""

from dataclasses import dataclass
from typing import List, Optional

import numpy as np
from pydrake.all import PiecewisePolynomial

from wimp import scene
from wimp.planning import diff_drive
from wimp.planning.planning_scene import PlanningScene
from wimp.planning.rrt_connect import (
    ConfigSpace,
    RRTConnectResult,
    path_length,
    rrt_connect,
    shortcut,
)

# D-002: RRT-Connect extension step in the cost metric (m).
RRT_STEP = 0.3
# Collision-check spacing along an edge, in the cost metric (0.03 m of
# driving, or 0.06 rad of turning).
BASE_EDGE_RESOLUTION = 0.03
# D-006: velocity limits used to time the path.
V_MAX = 0.5  # m/s
W_MAX = 0.8  # rad/s

# D-015: the base is a skid-steer / differential drive. Edges are
# turn-drive-turn motions; see wimp/planning/diff_drive.py.
wrap_angle = diff_drive.wrap_angle
base_distance = diff_drive.distance
base_interpolate = diff_drive.interpolate


@dataclass
class BasePlan:
    raw_path: List[np.ndarray]
    path: List[np.ndarray]  # shortcut, expanded into turn/drive primitives
    rrt: RRTConnectResult
    trajectory: PiecewisePolynomial  # (x, y, yaw) over time


class BasePlanner:
    def __init__(self, ps: Optional[PlanningScene] = None):
        self.ps = ps if ps is not None else PlanningScene()
        lo, hi = -scene.ROOM_HALF_EXTENT, scene.ROOM_HALF_EXTENT
        self.cspace = ConfigSpace(
            sample=lambda rng: np.array(
                [rng.uniform(lo, hi), rng.uniform(lo, hi), rng.uniform(-np.pi, np.pi)]
            ),
            distance=base_distance,
            interpolate=base_interpolate,
            is_free=self.is_free,
            edge_is_free=self.edge_is_free,
            edge_resolution=BASE_EDGE_RESOLUTION,
        )

    def default_base_q(self) -> np.ndarray:
        return self.ps.default_base_q()

    def is_free(self, q_base) -> bool:
        return self.ps.is_free(self.ps.full_q(q_base=q_base))

    def edge_is_free(self, a, b) -> bool:
        n = max(1, int(np.ceil(base_distance(a, b) / BASE_EDGE_RESOLUTION)))
        return self.ps.all_free(
            self.ps.full_q(q_base=base_interpolate(a, b, i / n)) for i in range(1, n + 1)
        )

    def plan(self, q_start, q_goal, seed=None, max_iterations=5000,
             shortcut_iterations=200, rng=None) -> Optional[BasePlan]:
        rng = rng if rng is not None else np.random.default_rng(seed)
        res = rrt_connect(np.asarray(q_start, float), np.asarray(q_goal, float),
                          self.cspace, step=RRT_STEP, max_iterations=max_iterations, rng=rng)
        if res.path is None:
            return None
        # Expand each edge into its turn / drive / turn primitives, so the
        # trajectory below executes exactly the collision-checked motion.
        short = diff_drive.expand_to_primitives(
            shortcut(res.path, self.cspace, shortcut_iterations, rng))
        return BasePlan(raw_path=res.path, path=short, rrt=res,
                        trajectory=time_parameterize(short))

    @staticmethod
    def describe(plan: BasePlan) -> str:
        return (
            f"RRT-Connect: {plan.rrt.iterations} iterations, "
            f"{len(plan.rrt.start_tree.nodes) + len(plan.rrt.goal_tree.nodes)} nodes | "
            f"raw path {len(plan.raw_path)} pts, length {path_length(plan.raw_path, base_distance):.2f} | "
            f"shortcut path {len(plan.path)} pts, length {path_length(plan.path, base_distance):.2f} | "
            f"duration {plan.trajectory.end_time():.1f} s"
        )


def stop_and_go_times(path, segment_time) -> np.ndarray:
    times = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        times.append(times[-1] + max(segment_time(a, b), 0.1))
    return np.array(times)


def time_parameterize(path, v_max=V_MAX, w_max=W_MAX) -> PiecewisePolynomial:
    """D-006: stop-at-each-waypoint cubic timing.

    Each primitive (a pure turn or a pure straight drive) uses
    s(tau) = 3 tau^2 - 2 tau^3 (zero velocity at both ends), so the robot
    stops between turning and driving and stays exactly on the
    collision-checked motion. Peak speed is 1.5 L / T, so T = 1.5 * max(L_xy / v_max,
    |dyaw| / w_max) respects the limits.
    """
    times = stop_and_go_times(
        path,
        lambda a, b: 1.5 * max(np.hypot(b[0] - a[0], b[1] - a[1]) / v_max,
                               abs(b[2] - a[2]) / w_max),
    )
    Q = np.array(path).T
    return PiecewisePolynomial.CubicHermite(times, Q, np.zeros_like(Q))
