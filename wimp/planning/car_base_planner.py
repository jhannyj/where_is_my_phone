"""Car-like base navigation planner: RRT-Connect in (x, y, yaw) with the arm
stowed, using Reeds-Shepp edges (forward/reverse arcs + straight lines, no
spinning in place).

Drop-in alternative to wimp/planning/base_planner.py.
"""

from typing import Optional

import numpy as np

from wimp import scene
from wimp.planning import reeds_shepp
from wimp.planning.base_planner import (
    BASE_EDGE_RESOLUTION,
    RRT_STEP,
    V_MAX,
    W_MAX,
    BasePlan,
)
from wimp.planning.planning_scene import PlanningScene
from wimp.planning.rrt_connect import ConfigSpace, rrt_connect, shortcut

wrap_angle = reeds_shepp.wrap_angle
base_distance = reeds_shepp.distance
base_interpolate = reeds_shepp.interpolate


def _polyline_length(path) -> float:
    return float(sum(np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:])))


class CarBasePlanner:
    def __init__(
        self,
        ps: Optional[PlanningScene] = None,
        turning_radius: float = reeds_shepp.TURNING_RADIUS,
    ):
        self.ps = ps if ps is not None else PlanningScene()
        self.turning_radius = turning_radius
        lo, hi = -scene.ROOM_HALF_EXTENT, scene.ROOM_HALF_EXTENT
        self.cspace = ConfigSpace(
            sample=lambda rng: np.array(
                [rng.uniform(lo, hi), rng.uniform(lo, hi), rng.uniform(-np.pi, np.pi)]
            ),
            distance=self.distance,
            interpolate=self.interpolate,
            is_free=self.is_free,
            edge_is_free=self.edge_is_free,
            edge_resolution=BASE_EDGE_RESOLUTION,
        )

    def distance(self, a, b) -> float:
        return reeds_shepp.distance(a, b, r=self.turning_radius)

    def interpolate(self, a, b, t: float) -> np.ndarray:
        return reeds_shepp.interpolate(a, b, t, r=self.turning_radius)

    def default_base_q(self) -> np.ndarray:
        return self.ps.default_base_q()

    def is_free(self, q_base) -> bool:
        return self.ps.is_free(self.ps.full_q(q_base=q_base))

    def edge_is_free(self, a, b) -> bool:
        segs = reeds_shepp.shortest_path(a, b, r=self.turning_radius)
        cur = np.asarray(a, float)
        qs = []
        for kind, length in segs:
            n = max(1, int(np.ceil(abs(length) / BASE_EDGE_RESOLUTION)))
            for i in range(1, n + 1):
                q_base = reeds_shepp._advance(cur, kind, length * (i / n), self.turning_radius)
                qs.append(self.ps.full_q(q_base=q_base))
            cur = reeds_shepp._advance(cur, kind, length, self.turning_radius)
        return self.ps.all_free(qs)

    def plan(
        self,
        q_start,
        q_goal,
        seed=None,
        max_iterations=5000,
        shortcut_iterations=200,
        rng=None,
    ) -> Optional[BasePlan]:
        rng = rng if rng is not None else np.random.default_rng(seed)
        res = rrt_connect(
            np.asarray(q_start, float),
            np.asarray(q_goal, float),
            self.cspace,
            step=RRT_STEP,
            max_iterations=max_iterations,
            rng=rng,
        )
        if res.path is None:
            return None

        short_waypoints = shortcut(res.path, self.cspace, shortcut_iterations, rng)
        raw_curve = reeds_shepp.sample_path(res.path, r=self.turning_radius)
        short_curve = reeds_shepp.sample_path(short_waypoints, r=self.turning_radius)
        traj = reeds_shepp.time_parameterize(
            short_waypoints, v_max=V_MAX, w_max=W_MAX, r=self.turning_radius
        )
        return BasePlan(raw_path=raw_curve, path=short_curve, rrt=res, trajectory=traj)

    @staticmethod
    def describe(plan: BasePlan) -> str:
        return (
            f"RRT-Connect (Reeds-Shepp): {plan.rrt.iterations} iterations, "
            f"{len(plan.rrt.start_tree.nodes) + len(plan.rrt.goal_tree.nodes)} nodes | "
            f"raw path {len(plan.raw_path)} pts, length {_polyline_length(plan.raw_path):.2f} m | "
            f"shortcut path {len(plan.path)} pts, length {_polyline_length(plan.path):.2f} m | "
            f"duration {plan.trajectory.end_time():.1f} s"
        )
