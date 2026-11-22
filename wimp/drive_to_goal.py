"""Drive the mobile manipulator to a goal pose in the living room, with full
knowledge of the scene.

    python -m wimp.drive_to_goal                    # default goal
    python -m wimp.drive_to_goal --goal 3.0 3.5 1.57 --seed 3

Pipeline: plan (x, y, yaw) path with RRT-Connect + shortcutting -> time it ->
simulate in Drake with joint PD control tracking the trajectory -> verify the
executed motion against the collision checker -> save a Meshcat replay to
outputs/drive_to_goal.html.
"""

import argparse
import os
import time

import numpy as np
from pydrake.all import Rgba, Simulator

from wimp import scene, sim
from wimp.planning.base_planner import BasePlanner, wrap_angle
from wimp.planning.planning_scene import ENV_PADDING
from wimp.planning.car_base_planner import CarBasePlanner

# Default goal: east side of the room, south of the coffee table, facing +x.
DEFAULT_GOAL = (3.2, -2.0, 0.0)


def draw_plan(meshcat, plan, z=0.03):
    """RRT trees (thin), raw path (orange) and shortcut path (green)."""
    for name, tree, color in [
        ("start_tree", plan.rrt.start_tree, Rgba(0.2, 0.4, 1.0, 0.5)),
        ("goal_tree", plan.rrt.goal_tree, Rgba(1.0, 0.3, 0.3, 0.5)),
    ]:
        edges = [(tree.nodes[p], tree.nodes[i]) for i, p in enumerate(tree.parents) if p >= 0]
        if edges:
            s = np.array([[*a[:2], z] for a, _ in edges]).T
            e = np.array([[*b[:2], z] for _, b in edges]).T
            meshcat.SetLineSegments(f"plan/{name}", s, e, 1.0, color)
    sim.draw_polyline(meshcat, plan.raw_path, "plan/raw_path", z + 0.01, 3.0, Rgba(1, 0.6, 0, 1))
    sim.draw_polyline(meshcat, plan.path, "plan/path", z + 0.02, 6.0, Rgba(0.1, 0.8, 0.2, 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--goal", type=float, nargs=3, default=DEFAULT_GOAL,
                    metavar=("X", "Y", "YAW"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--html", default=os.path.join(scene.REPO_ROOT, "outputs", "drive_to_goal.html"))
    ap.add_argument("--wait", action="store_true", help="keep Meshcat open until Enter")
    ap.add_argument("--car", action="store_true", help="use Reeds-Shepp car steering")
    ap.add_argument("--radius", type=float, default=0.6, help="car turning radius (m)")
    args = ap.parse_args()

    planner = CarBasePlanner(turning_radius=args.radius) if args.car else BasePlanner()
    ps = planner.ps
    q_start, q_goal = planner.default_base_q(), np.array(args.goal)
    print(f"start (x, y, yaw) = {np.round(q_start, 3)}, goal = {np.round(q_goal, 3)}")
    t0 = time.time()
    plan = planner.plan(q_start, q_goal, seed=args.seed)
    if plan is None:
        raise SystemExit("RRT-Connect failed to find a path")
    print(f"planned in {time.time() - t0:.2f} s | {BasePlanner.describe(plan)}")

    T = plan.trajectory.end_time() + 1.5  # extra time to settle at the goal
    q_end = plan.trajectory.value(plan.trajectory.end_time()).ravel()  # yaw unwrapped
    base_traj = sim.concat([plan.trajectory, sim.hold(q_end, T - plan.trajectory.end_time())])
    meshcat = scene.start_meshcat()
    diagram, plant, visualizer, logs = sim.build_sim(
        meshcat, base_traj, sim.hold(ps.arm_stow(), T), sim.hold(sim.GRIPPER_OPEN, T))
    draw_plan(meshcat, plan)

    simulator = Simulator(diagram)
    simulator.Initialize()
    visualizer.StartRecording(False)
    t0 = time.time()
    simulator.AdvanceTo(T)
    visualizer.StopRecording()
    visualizer.PublishRecording()
    print(f"simulated {T:.1f} s in {time.time() - t0:.1f} s wall time")

    log = logs["base"].FindLog(simulator.get_context())
    q_exec = log.data()[:3, :]
    desired = np.hstack([base_traj.value(t) for t in log.sample_times()])
    track_err = np.max(np.hypot(*(q_exec[:2] - desired[:2])))
    final = q_exec[:, -1]
    print(f"final pose {np.round(final, 3)} | goal error "
          f"{100 * np.hypot(*(final[:2] - q_goal[:2])):.2f} cm, "
          f"{np.rad2deg(abs(wrap_angle(final[2] - q_goal[2]))):.2f} deg "
          f"| max xy tracking error {track_err:.4f} m")
    # D-015: a skid-steer base must not slide sideways. Velocity component
    # perpendicular to the chassis heading:
    v = log.data()[3:5, :]
    yaw = q_exec[2]
    v_side = -np.sin(yaw) * v[0] + np.cos(yaw) * v[1]
    print(f"max sideways speed {np.max(np.abs(v_side)):.4f} m/s "
          f"(max speed {np.max(np.hypot(*v)):.2f} m/s)")
    stride = max(1, q_exec.shape[1] // 400)
    exec_free = all(planner.is_free(q) for q in q_exec[:, ::stride].T)
    print(f"executed motion keeps {ENV_PADDING} m clearance: {exec_free}")

    os.makedirs(os.path.dirname(args.html), exist_ok=True)
    with open(args.html, "w") as f:
        f.write(meshcat.StaticHtml())
    print(f"replay saved to {args.html}")
    if args.wait:
        input("Meshcat is live; press Enter to exit.")


if __name__ == "__main__":
    main()
