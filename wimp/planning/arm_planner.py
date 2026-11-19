"""Collision-free arm motion in joint space (D-013): RRT-Connect over the 7
iiwa joints with the base fixed, reusing the same planner as the base.
Most of the time the straight joint-space line is already free, so the
first CONNECT succeeds immediately."""

from typing import List, Optional

import numpy as np
from pydrake.all import PiecewisePolynomial

from wimp.planning.base_planner import stop_and_go_times
from wimp.planning.planning_scene import PlanningScene
from wimp.planning.rrt_connect import ConfigSpace, rrt_connect, shortcut

JOINT_STEP = 0.05  # rad, edge collision-check resolution
QDOT_MAX = 0.8  # rad/s per joint, for timing


def plan_arm_path(ps: PlanningScene, q_base, q_start, q_goal, seed=0,
                  max_iterations=3000) -> Optional[List[np.ndarray]]:
    lo, hi = ps.arm_limits()

    def is_free(q):
        return ps.is_free(ps.full_q(q_base, q))

    def edge_free(a, b):
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / JOINT_STEP)))
        return ps.all_free(ps.full_q(q_base, a + (b - a) * i / n) for i in range(1, n + 1))

    cs = ConfigSpace(
        sample=lambda rng: rng.uniform(lo, hi),
        distance=lambda a, b: float(np.linalg.norm(b - a)),
        interpolate=lambda a, b, t: a + t * (b - a),
        is_free=is_free,
        edge_is_free=edge_free,
    )
    rng = np.random.default_rng(seed)
    res = rrt_connect(np.asarray(q_start, float), np.asarray(q_goal, float), cs,
                      step=0.5, max_iterations=max_iterations, rng=rng)
    if res.path is None:
        return None
    return shortcut(res.path, cs, 100, rng)


def time_arm_path(path, qdot_max=QDOT_MAX) -> PiecewisePolynomial:
    """Same stop-at-waypoint cubic timing as the base (D-006)."""
    times = stop_and_go_times(path, lambda a, b: 1.5 * np.max(np.abs(b - a)) / qdot_max)
    Q = np.array(path).T
    return PiecewisePolynomial.CubicHermite(times, Q, np.zeros_like(Q))
