"""Where to park the base so the arm can grasp the phone (D-014).

Score each candidate base pose by its *expected reachability* over the
robot's belief about where the phone is:

    reach(b) = (1/K) * sum_k 1[ some grasp of particle k is feasible from b ]

where a grasp is feasible if IK finds collision-free pre-grasp and grasp
configurations and the straight-line approach between them is
collision-free. Among the best-scoring bases, prefer the one closest to the
robot (less driving), and that the base planner can actually drive to.
Reach scores within TIE_TOLERANCE of each other count as ties.

With exact knowledge (one particle) this is "sample, check IK, take the
closest". Once vision/audio give a spread-out belief, the same code picks
the spot that covers the most likely phone locations. A sharper belief
(audio) should therefore mean fewer re-parks.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np
from pydrake.all import RigidTransform

from wimp.grasping.antipodal import Grasp
from wimp.grasping.kinematics import diff_ik_line, solve_arm_ik
from wimp.planning.base_planner import base_distance
from wimp.planning.planning_scene import PlanningScene

PREGRASP_STANDOFF = 0.10  # m, back off along the approach axis
APPROACH_TIME = 2.0  # s
# D-014 refinements (2026-11-18): reach differences within TIE_TOLERANCE
# count as ties (closer base wins); below MIN_REACH_TO_COMMIT the robot
# parks, re-observes and re-plans.
TIE_TOLERANCE = 0.1
MIN_REACH_TO_COMMIT = 0.6


@dataclass
class GraspPlan:
    q_base: np.ndarray
    grasp: Grasp
    q_pregrasp: np.ndarray
    q_grasp: np.ndarray
    approach: List[np.ndarray]  # arm configs every 10 ms, pregrasp -> grasp


# Distance band (m) from the shoulder (iiwa joint 2) to the gripper frame
# that the iiwa 14 can possibly reach: upper arm 0.42 + forearm 0.40 + wrist
# to flange ~0.13 + flange to gripper frame 0.09 ~ 1.04, padded to 1.10 so
# the band stays a safe necessary condition. Grasps outside the band are
# skipped without an IK solve.
REACH_MIN, REACH_MAX = 0.15, 1.10
MAX_GRASPS_PER_BASE = 3


def shoulder_position(ps: PlanningScene, q_base) -> np.ndarray:
    ps.plant.SetPositions(ps.plant_context, ps.full_q(q_base=q_base))
    link2 = ps.plant.GetBodyByName("iiwa_link_2")
    return ps.plant.EvalBodyPoseInWorld(ps.plant_context, link2).translation()


def plan_grasp_from_base(ps: PlanningScene, q_base, grasps: Sequence[Grasp],
                         max_grasps: int = MAX_GRASPS_PER_BASE) -> Optional[GraspPlan]:
    """Try the best `max_grasps` grasps that pass the reach-band prefilter."""
    p_sh = shoulder_position(ps, q_base)
    in_band = [g for g in grasps
               if REACH_MIN <= np.linalg.norm(g.X_WG.translation() - p_sh) <= REACH_MAX
               and np.linalg.norm(g.pregrasp(PREGRASP_STANDOFF).translation() - p_sh) <= REACH_MAX]
    q_stow = ps.arm_stow()
    for g in in_band[:max_grasps]:
        q_pre = solve_arm_ik(ps, q_base, g.pregrasp(PREGRASP_STANDOFF), q_stow)
        if q_pre is None or not ps.is_free(ps.full_q(q_base, q_pre)):
            continue
        q_g = solve_arm_ik(ps, q_base, g.X_WG, q_pre)
        if q_g is None or not ps.is_free(ps.full_q(q_base, q_g)):
            continue
        approach = diff_ik_line(ps, q_base, q_pre, g.X_WG, APPROACH_TIME)
        if not ps.all_free(ps.full_q(q_base, q) for q in approach[::5]):
            continue
        return GraspPlan(np.asarray(q_base, float), g, q_pre, q_g, approach)
    return None


def candidate_bases(target_xy, radii=(0.6, 0.7, 0.8, 0.9), num_angles=24,
                    yaw_offsets=(0.0, np.pi / 2, -np.pi / 2)) -> List[np.ndarray]:
    """Ring of base poses around the target. yaw offset 0 = chassis front
    (+x) facing the target; +-90 deg = target off to the side."""
    out = []
    for r in radii:
        for phi in np.linspace(-np.pi, np.pi, num_angles, endpoint=False):
            x, y = target_xy[0] + r * np.cos(phi), target_xy[1] + r * np.sin(phi)
            facing = np.arctan2(target_xy[1] - y, target_xy[0] - x)
            out += [np.array([x, y, facing + off]) for off in yaw_offsets]
    return out


def choose_base(
    ps: PlanningScene,
    particles: Sequence[RigidTransform],
    grasps_per_particle: Sequence[Sequence[Grasp]],
    q_base_now,
    is_base_free,
    is_base_reachable=lambda b: True,
    good_enough: float = 1.0,
    max_evaluated: int = 40,
):
    """Return (best GraspPlan for the mean particle, reach score, stats).

    Policy for low reach (caller's job, D-014): if the best score is below
    MIN_REACH_TO_COMMIT, still park there, re-observe, update the belief, and
    re-plan; each re-plan counts as a re-park in the experiment logs.

    `is_base_reachable(b)` (e.g. "the base planner finds a path there") is
    only called for a candidate that would become the new best, because
    a collision-free spot can still be an enclosed pocket the base cannot
    drive into."""
    center = np.mean([X.translation()[:2] for X in particles], axis=0)
    cands = [b for b in candidate_bases(center) if is_base_free(b)]
    cands.sort(key=lambda b: base_distance(q_base_now, b))
    best, best_score, evaluated = None, -1.0, 0
    for b in cands[:max_evaluated]:
        evaluated += 1
        plans = [plan_grasp_from_base(ps, b, g) for g in grasps_per_particle]
        score = float(np.mean([p is not None for p in plans]))
        # Candidates are sorted by travel distance, so a farther base must
        # beat the current best by more than TIE_TOLERANCE to replace it.
        if (score > best_score + (TIE_TOLERANCE if best is not None else 0.0)
                and plans[0] is not None and is_base_reachable(b)):
            best, best_score = plans[0], score
        if best_score >= good_enough - TIE_TOLERANCE:
            break
    stats = dict(free_candidates=len(cands), evaluated=evaluated)
    return best, best_score, stats
