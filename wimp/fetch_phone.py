"""Known-pose fetch: drive to the phone, grasp it, and lift it 15 cm.

    python -m wimp.fetch_phone [--seed 0] [--wait]

The phone's pose is taken from the simulator (privileged information), so
this exercises the manipulation pipeline before perception exists:

  1. antipodal grasp candidates on the phone's surface, filtered by a
     floating-gripper collision check                       (D-012)
  2. base placement by expected reachability (IK checked)   (D-014)
  3. RRT-Connect base path to that pose                     (D-002)
  4. RRT-Connect arm path: stow -> pre-grasp                (D-013)
  5. differential IK: straight-line approach, then lift     (D-013)
  6. close the gripper, simulate everything, check success:
     phone lifted >= 10 cm and still between the fingers.
"""

import argparse
import os
import time

import numpy as np
from pydrake.all import RigidTransform, Rgba, Simulator

from wimp import scene, sim
from wimp.grasping.antipodal import PAD_CENTER_Y, antipodal_grasps, sample_box_surface
from wimp.grasping.gripper_checker import GripperChecker
from wimp.grasping.base_placement import APPROACH_TIME, choose_base
from wimp.grasping.kinematics import diff_ik_line, pose_error
from wimp.planning.arm_planner import plan_arm_path, time_arm_path
from wimp.planning.base_planner import BasePlanner
from wimp.planning.planning_scene import PlanningScene
from wimp.planning.car_base_planner import CarBasePlanner

PHONE_SIZE = (0.075, 0.15, 0.015)
LIFT_HEIGHT = 0.15
DT = 0.01


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--html", default=os.path.join(scene.REPO_ROOT, "outputs", "fetch_phone.html"))
    ap.add_argument("--wait", action="store_true", help="keep Meshcat open until Enter")
    ap.add_argument("--verbose", action="store_true", help="print a grasp timeline")
    ap.add_argument("--phone", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"),
                    help="override the phone pose (lying flat on the coffee table)")
    ap.add_argument("--car", action="store_true", help="use Reeds-Shepp car steering")
    ap.add_argument("--radius", type=float, default=0.6, help="car turning radius (m)")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    t_all = time.time()

    phone_pose = scene.phone_on_table(*args.phone) if args.phone else None
    ps = PlanningScene(phone_pose=phone_pose)
    # base_planner = BasePlanner(ps)
    base_planner = CarBasePlanner(ps, turning_radius=args.radius) if args.car else BasePlanner(ps)
    q_base0 = ps.default_base_q()
    X_WO = ps.phone_pose()  # privileged
    print(f"phone at {np.round(X_WO.translation(), 3)}")

    # 1. grasp candidates
    P, N = sample_box_surface(X_WO, PHONE_SIZE, 3000, rng)
    candidates = antipodal_grasps(P, N, rng)
    t0 = time.time()
    grasps = GripperChecker(phone_pose).filter(candidates)
    print(f"{len(candidates)} antipodal candidates, {len(grasps)} with a collision-free "
          f"floating gripper ({time.time() - t0:.1f} s); best score {grasps[0].score:.2f}, "
          f"width {grasps[0].width * 100:.1f} cm")

    # 2. base placement
    t0 = time.time()
    base_plans = {}

    def reachable(b):  # plan the drive now and keep it for step 3
        base_plans[tuple(b)] = base_planner.plan(q_base0, b, rng=rng, max_iterations=3000)
        return base_plans[tuple(b)] is not None

    plan, reach, stats = choose_base(ps, [X_WO], [grasps], q_base0, base_planner.is_free,
                                     is_base_reachable=reachable)
    if plan is None:
        raise SystemExit(f"no reachable base pose found ({stats})")
    print(f"base pose {np.round(plan.q_base, 3)} reach={reach:.2f} "
          f"({stats['evaluated']} of {stats['free_candidates']} free candidates evaluated, "
          f"{time.time() - t0:.1f} s)")

    # 3. base path
    base_plan = base_plans[tuple(plan.q_base)]
    if base_plan is None:
        raise SystemExit("base path planning failed")
    print(base_planner.describe(base_plan))

    # 4. arm path stow -> pregrasp
    arm_path = plan_arm_path(ps, plan.q_base, ps.arm_stow(), plan.q_pregrasp, seed=args.seed)
    if arm_path is None:
        raise SystemExit("arm path planning failed")
    arm_to_pre = time_arm_path(arm_path)
    print(f"arm path: {len(arm_path)} waypoints, {arm_to_pre.end_time():.1f} s")

    # 5. lift with diff IK (approach was computed during base placement)
    q_at_grasp = plan.approach[-1]
    X_lift = RigidTransform(plan.grasp.X_WG.rotation(),
                            plan.grasp.X_WG.translation() + [0, 0, LIFT_HEIGHT])
    lift = diff_ik_line(ps, plan.q_base, q_at_grasp, X_lift, 2.5)

    # 6. assemble the timeline
    T_drive = base_plan.trajectory.end_time()
    T_close, T_settle = 1.5, 1.5
    arm_traj = sim.concat([
        sim.hold(ps.arm_stow(), T_drive),
        arm_to_pre,
        sim.smooth_samples(plan.approach, DT),
        sim.hold(q_at_grasp, T_close),
        sim.smooth_samples(lift, DT),
        sim.hold(lift[-1], T_settle),
    ])
    T_end = arm_traj.end_time()
    # Hold the trajectory's own final value: its yaw is unwrapped (it can
    # differ from plan.q_base by 2*pi), and a jump would spin the base.
    q_base_end = base_plan.trajectory.value(T_drive).ravel()
    base_traj = sim.concat([base_plan.trajectory, sim.hold(q_base_end, T_end - T_drive)])
    t_close = T_drive + arm_to_pre.end_time() + APPROACH_TIME
    gripper_traj = sim.concat([
        sim.hold(sim.GRIPPER_OPEN, t_close),
        sim.ramp(sim.GRIPPER_OPEN, sim.GRIPPER_CLOSED, 1.0),
        sim.hold(sim.GRIPPER_CLOSED, T_end - t_close - 1.0),
    ])
    print(f"planning total {time.time() - t_all:.1f} s; motion lasts {T_end:.1f} s")

    # simulate
    meshcat = scene.start_meshcat()
    diagram, plant, visualizer, logs = sim.build_sim(
        meshcat, base_traj, arm_traj, gripper_traj, phone_pose=phone_pose)
    sim.draw_polyline(meshcat, base_plan.path, "plan/base_path", 0.03, 6.0, Rgba(0.1, 0.8, 0.2, 1))
    simulator = Simulator(diagram)
    simulator.Initialize()
    visualizer.StartRecording(False)
    t0 = time.time()
    simulator.AdvanceTo(T_end)
    visualizer.StopRecording()
    visualizer.PublishRecording()
    print(f"simulated {T_end:.1f} s in {time.time() - t0:.1f} s wall time")

    # evaluate
    ctx = simulator.get_context()
    # Where was the gripper, relative to the plan, when the fingers closed?
    k = np.searchsorted(logs["arm"].FindLog(ctx).sample_times(), t_close)
    q_arm_c = logs["arm"].FindLog(ctx).data()[:7, k]
    q_base_c = logs["base"].FindLog(ctx).data()[:3, k]
    dp, dR = pose_error(ps.gripper_pose(ps.full_q(q_base_c, q_arm_c)), plan.grasp.X_WG)
    dp_plan, _ = pose_error(ps.gripper_pose(ps.full_q(plan.q_base, q_at_grasp)), plan.grasp.X_WG)
    ph = logs["phone"].FindLog(ctx).data()[4:7, k]
    print(f"at close: gripper error vs planned grasp {100 * dp:.1f} cm / {np.rad2deg(dR):.1f} deg "
          f"(diff-IK end error {100 * dp_plan:.1f} cm) | phone moved "
          f"{100 * np.linalg.norm(ph - X_WO.translation()):.1f} cm | base error "
          f"{100 * np.linalg.norm(q_base_c[:2] - plan.q_base[:2]):.1f} cm")
    if args.verbose:
        blog = logs["base"].FindLog(ctx)
        des = np.hstack([base_traj.value(t) for t in blog.sample_times()])
        err = np.hypot(*(blog.data()[:2] - des[:2]))
        k_bad = np.argmax(err > 0.01) if np.any(err > 0.01) else None
        print(f"  base: max tracking error {100 * err.max():.1f} cm"
              + (f", first > 1 cm at t={blog.sample_times()[k_bad]:.2f} s, desired "
                 f"{np.round(des[:, k_bad], 3)}, actual {np.round(blog.data()[:3, k_bad], 3)}"
                 if k_bad is not None else ""))
        tl = logs["phone"].FindLog(ctx).sample_times()
        for t in np.arange(t_close - 0.5, T_end, 0.5):
            i = np.searchsorted(tl, t)
            qa = logs["arm"].FindLog(ctx).data()[:7, i]
            qb = logs["base"].FindLog(ctx).data()[:3, i]
            qw = logs["wsg"].FindLog(ctx).data()[:2, i]
            pph = logs["phone"].FindLog(ctx).data()[4:7, i]
            pads = ps.gripper_pose(ps.full_q(qb, qa)).multiply([0, PAD_CENTER_Y, 0])
            print(f"  t={t:5.1f} fingers {np.round(qw * 100, 2)} cm | phone z {pph[2]:.4f} "
                  f"| phone-pads {np.round(100 * (pph - pads), 1)} cm")
    phone_log = logs["phone"].FindLog(ctx).data()  # [quat(4), p(3), w(3), v(3)]
    z0, z1 = phone_log[6, 0], phone_log[6, -1]
    pc = plant.GetMyContextFromRoot(ctx)
    X_WG = plant.EvalBodyPoseInWorld(pc, plant.GetBodyByName("body"))
    p_pads = X_WG.multiply([0, PAD_CENTER_Y, 0])
    p_phone = phone_log[4:7, -1]
    in_hand = np.linalg.norm(p_phone - p_pads) < 0.03
    finger_gap = np.diff(logs["wsg"].FindLog(ctx).data()[:2, -1])[0]
    lifted = (z1 - z0) >= 0.10
    print(f"phone z {z0:.3f} -> {z1:.3f} (lift {100 * (z1 - z0):.1f} cm) | "
          f"phone-to-pads {100 * np.linalg.norm(p_phone - p_pads):.1f} cm | "
          f"finger joint gap {100 * finger_gap:.1f} cm")
    print("SUCCESS" if (lifted and in_hand) else "FAILURE",
          f"(lifted >= 10 cm: {lifted}, in hand: {in_hand})")

    os.makedirs(os.path.dirname(args.html), exist_ok=True)
    with open(args.html, "w") as f:
        f.write(meshcat.StaticHtml())
    print(f"replay saved to {args.html}")
    if args.wait:
        input("Meshcat is live; press Enter to exit.")


if __name__ == "__main__":
    main()
