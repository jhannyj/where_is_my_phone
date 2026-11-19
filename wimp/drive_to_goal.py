"""Drive the mobile manipulator to a goal pose in the living
room, with full knowledge of the scene.

    python -m wimp.drive_to_goal                    # default goal near phone
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
from pydrake.all import (
    AddMultibodyPlant,
    ConstantVectorSource,
    DiagramBuilder,
    JointActuatorIndex,
    MeshcatVisualizer,
    MultibodyPlantConfig,
    Parser,
    PdControllerGains,
    Rgba,
    Simulator,
    TrajectorySource,
    VectorLogSink,
)

from wimp import scene
from wimp.planning.base_planner import ENV_PADDING, BasePlanner, base_distance

# Default goal: east side of the room, south of the coffee table, facing +x.
DEFAULT_GOAL = (3.2, -2.0, 0.0)

# D-007: joint-level PD gains (placeholder controller).
BASE_XY_GAINS = PdControllerGains(p=8000.0, d=1600.0)
BASE_YAW_GAINS = PdControllerGains(p=3000.0, d=400.0)
ARM_GAINS = PdControllerGains(p=1000.0, d=100.0)
GRIPPER_GAINS = PdControllerGains(p=200.0, d=5.0)


def draw_plan(meshcat, plan, z=0.03):
    """RRT trees (thin), raw path (orange) and shortcut path (green)."""

    def segments(tree):
        starts, ends = [], []
        for i, p in enumerate(tree.parents):
            if p >= 0:
                starts.append([*tree.nodes[p][:2], z])
                ends.append([*tree.nodes[i][:2], z])
        return np.array(starts).T, np.array(ends).T

    for name, tree, color in [
        ("start_tree", plan.rrt.start_tree, Rgba(0.2, 0.4, 1.0, 0.5)),
        ("goal_tree", plan.rrt.goal_tree, Rgba(1.0, 0.3, 0.3, 0.5)),
    ]:
        if len(tree.nodes) > 1:
            s, e = segments(tree)
            meshcat.SetLineSegments(f"plan/{name}", s, e, 1.0, color)

    def polyline(path, z):
        return np.array([[q[0], q[1], z] for q in path]).T

    meshcat.SetLine("plan/raw_path", polyline(plan.raw_path, z + 0.01), 3.0, Rgba(1, 0.6, 0, 1))
    meshcat.SetLine("plan/path", polyline(plan.path, z + 0.02), 6.0, Rgba(0.1, 0.8, 0.2, 1))


def build_sim(meshcat, base_traj, q_arm):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(time_step=0.002, discrete_contact_approximation="sap"),
        builder,
    )
    scene.load_scene(Parser(plant))
    base = plant.GetModelInstanceByName(scene.BASE_INSTANCE)
    arm = plant.GetModelInstanceByName(scene.ARM_INSTANCE)
    wsg = plant.GetModelInstanceByName(scene.GRIPPER_INSTANCE)

    # D-007: joint PD + ideal gravity compensation (gravity off for the robot).
    for inst in scene.robot_instances(plant):
        plant.set_gravity_enabled(inst, False)
    # (GetJointActuatorIndices(model) is empty before Finalize, so loop over all.)
    for i in range(plant.num_actuators()):
        act = plant.get_mutable_joint_actuator(JointActuatorIndex(i))
        inst = act.model_instance()
        if inst == base:
            g = BASE_YAW_GAINS if act.joint().name() == "yaw_joint" else BASE_XY_GAINS
        elif inst == arm:
            g = ARM_GAINS
        elif inst == wsg:
            g = GRIPPER_GAINS
        else:
            continue
        act.set_controller_gains(g)
    plant.Finalize()

    base_src = builder.AddSystem(TrajectorySource(base_traj, output_derivative_order=1))
    builder.Connect(base_src.get_output_port(), plant.get_desired_state_input_port(base))
    arm_src = builder.AddSystem(ConstantVectorSource(np.concatenate([q_arm, np.zeros(7)])))
    builder.Connect(arm_src.get_output_port(), plant.get_desired_state_input_port(arm))
    q_wsg = plant.GetDefaultPositions(wsg)
    wsg_src = builder.AddSystem(ConstantVectorSource(np.concatenate([q_wsg, np.zeros_like(q_wsg)])))
    builder.Connect(wsg_src.get_output_port(), plant.get_desired_state_input_port(wsg))

    logger = builder.AddSystem(VectorLogSink(plant.num_multibody_states(base)))
    builder.Connect(plant.get_state_output_port(base), logger.get_input_port())

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), visualizer, logger


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--goal", type=float, nargs=3, default=DEFAULT_GOAL,
                    metavar=("X", "Y", "YAW"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--html", default=os.path.join(scene.REPO_ROOT, "outputs", "drive_to_goal.html"))
    ap.add_argument("--realtime", action="store_true", help="simulate at real-time speed")
    ap.add_argument("--wait", action="store_true", help="keep Meshcat open until Enter")
    args = ap.parse_args()

    # ---------------- plan ----------------
    planner = BasePlanner()
    q_start = planner.default_base_q()
    q_goal = np.array(args.goal)
    print(f"start (x, y, yaw) = {np.round(q_start, 3)}, goal = {np.round(q_goal, 3)}")
    t0 = time.time()
    plan = planner.plan(q_start, q_goal, seed=args.seed)
    if plan is None:
        raise SystemExit("RRT-Connect failed to find a path")
    print(f"planned in {time.time() - t0:.2f} s | {BasePlanner.describe(plan)}")

    # ---------------- simulate ----------------
    meshcat = scene.start_meshcat()
    q_arm = planner.q_template[planner.idx.arm_q]
    diagram, visualizer, logger = build_sim(meshcat, plan.trajectory, q_arm)
    draw_plan(meshcat, plan)

    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0 if args.realtime else 0.0)
    simulator.Initialize()
    visualizer.StartRecording(False)
    T = plan.trajectory.end_time() + 1.5  # extra time to settle at the goal
    t0 = time.time()
    simulator.AdvanceTo(T)
    visualizer.StopRecording()
    visualizer.PublishRecording()
    print(f"simulated {T:.1f} s in {time.time() - t0:.1f} s wall time")

    # ---------------- evaluate ----------------
    log = logger.FindLog(simulator.get_context())
    q_exec = log.data()[:3, :]  # (x, y, yaw) over time
    desired = np.hstack([plan.trajectory.value(min(t, plan.trajectory.end_time())) for t in log.sample_times()])
    track_err = np.max(np.hypot(*(q_exec[:2] - desired[:2])))
    final = q_exec[:, -1]
    print(f"final pose {np.round(final, 3)} | goal error {base_distance(final, q_goal):.4f} "
          f"| max xy tracking error {track_err:.4f} m")
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
