"""Arm kinematics: optimization-based IK and differential IK (D-013).

- solve_arm_ik: Drake InverseKinematics (a nonlinear program solved with
  SNOPT/IPOPT) for the 7 iiwa joints with the base fixed. Constraints: the
  gripper position within `pos_tol` of the target, orientation within
  `ang_tol`, joint limits. Cost: stay close to a seed posture. Collisions are
  NOT a constraint here; callers check the solution with the collision
  checker and reject it if it collides (simpler and more robust than
  minimum-distance constraints, which also see the phone touching the table).
- diff_ik_line: moves the gripper along a straight Cartesian line by
  integrating damped-least-squares differential IK,
      qdot = J^T (J J^T + lambda^2 I)^-1 V,
  where V is a feed-forward twist plus a proportional correction on pose
  error. Used for the final approach and the lift, where we want a straight
  line in Cartesian space, not in joint space.
"""

from typing import List, Optional

import numpy as np
from pydrake.all import (
    InverseKinematics,
    JacobianWrtVariable,
    PiecewisePose,
    RigidTransform,
    RotationMatrix,
    Solve,
)

from wimp.planning.planning_scene import PlanningScene


def arm_velocity_indices(ps: PlanningScene) -> np.ndarray:
    from wimp import scene

    return np.array([j.velocity_start() for j in scene.arm_joints(ps.plant)])


def solve_arm_ik(
    ps: PlanningScene,
    q_base,
    X_WG: RigidTransform,
    q_seed,
    pos_tol: float = 0.0005,
    ang_tol: float = 0.005,
    restarts: int = 3,
    rng=None,
) -> Optional[np.ndarray]:
    plant = ps.plant
    ik = InverseKinematics(plant, with_joint_limits=True)
    q = ik.q()
    prog = ik.get_mutable_prog()

    q_fixed = ps.full_q(q_base=q_base)
    others = np.setdiff1d(np.arange(plant.num_positions()), ps.idx.arm_q)
    prog.AddBoundingBoxConstraint(q_fixed[others], q_fixed[others], q[others])

    p = X_WG.translation()
    ik.AddPositionConstraint(ps.gripper_frame, [0, 0, 0], plant.world_frame(),
                             p - pos_tol, p + pos_tol)
    ik.AddOrientationConstraint(plant.world_frame(), X_WG.rotation(),
                                ps.gripper_frame, RotationMatrix(), ang_tol)
    q_nominal = q_fixed.copy()
    q_nominal[ps.idx.arm_q] = q_seed
    prog.AddQuadraticErrorCost(np.eye(len(ps.idx.arm_q)), np.asarray(q_seed),
                               q[ps.idx.arm_q])

    rng = rng if rng is not None else np.random.default_rng(0)
    lo, hi = ps.arm_limits()
    for k in range(restarts):
        guess = q_nominal.copy()
        if k > 0:  # random restart near the seed
            guess[ps.idx.arm_q] = np.clip(q_seed + rng.normal(0, 0.5, len(q_seed)), lo, hi)
        prog.SetInitialGuess(q, guess)
        result = Solve(prog)
        if result.is_success():
            return result.GetSolution(q)[ps.idx.arm_q]
    return None


def diff_ik_line(
    ps: PlanningScene,
    q_base,
    q_arm_start,
    X_WG_goal: RigidTransform,
    duration: float,
    dt: float = 0.01,
    kp: float = 5.0,
    damping: float = 0.01,
    qdot_max: float = 1.5,
) -> List[np.ndarray]:
    """Arm configurations (every dt) that move the gripper from its pose at
    q_arm_start to X_WG_goal along a straight line (orientation slerped),
    with smooth s(tau) = 3 tau^2 - 2 tau^3 timing."""
    plant, ctx = ps.plant, ps.plant_context
    v_idx = arm_velocity_indices(ps)
    q_arm = np.asarray(q_arm_start, float).copy()
    X_start = ps.gripper_pose(ps.full_q(q_base, q_arm))
    n = int(np.ceil(duration / dt))
    s = lambda tau: 3 * tau**2 - 2 * tau**3
    line = PiecewisePose.MakeLinear([0.0, 1.0], [X_start, X_WG_goal])
    out = [q_arm.copy()]
    lo, hi = ps.arm_limits()
    for k in range(1, n + 1):
        s0, s1 = s((k - 1) / n), s(k / n)
        X_des = line.GetPose(s1)
        # feed-forward twist (finite difference of the path) + feedback
        V_ff = line.GetVelocity(0.5 * (s0 + s1)) * (s1 - s0) / dt
        q_full = ps.full_q(q_base, q_arm)
        plant.SetPositions(ctx, q_full)
        X_now = plant.CalcRelativeTransform(ctx, plant.world_frame(), ps.gripper_frame)
        err_p = X_des.translation() - X_now.translation()
        err_R = (X_des.rotation().multiply(X_now.rotation().inverse())).ToAngleAxis()
        err_w = err_R.axis() * err_R.angle()
        V = np.asarray(V_ff).ravel() + kp * np.concatenate([err_w, err_p])
        J = plant.CalcJacobianSpatialVelocity(
            ctx, JacobianWrtVariable.kV, ps.gripper_frame, [0, 0, 0],
            plant.world_frame(), plant.world_frame(),
        )[:, v_idx]
        qdot = J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), V)
        qdot = np.clip(qdot, -qdot_max, qdot_max)
        q_arm = np.clip(q_arm + dt * qdot, lo, hi)
        out.append(q_arm.copy())
    return out


def pose_error(X_a: RigidTransform, X_b: RigidTransform):
    """(position error m, rotation error rad)."""
    dp = np.linalg.norm(X_a.translation() - X_b.translation())
    dR = X_a.rotation().inverse().multiply(X_b.rotation()).ToAngleAxis().angle()
    return float(dp), float(abs(dR))
