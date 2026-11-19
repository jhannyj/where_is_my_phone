"""Physics simulation of the robot tracking joint-space reference
trajectories (D-007: joint PD on every actuator, ideal gravity compensation
for the robot). The phone and the furniture are simulated normally."""

import copy
from typing import Sequence

import numpy as np
from pydrake.all import (
    AddMultibodyPlant,
    DiagramBuilder,
    JointActuatorIndex,
    MeshcatVisualizer,
    MultibodyPlantConfig,
    Parser,
    PdControllerGains,
    PiecewisePolynomial,
    Rgba,
    TrajectorySource,
    VectorLogSink,
)

from wimp import scene

BASE_XY_GAINS = PdControllerGains(p=8000.0, d=1600.0)
BASE_YAW_GAINS = PdControllerGains(p=3000.0, d=400.0)
ARM_GAINS = PdControllerGains(p=1000.0, d=100.0)
# Closing the fingers past the object makes the PD act as a force
# controller: squeeze ~ p * (penetration of the setpoint) = 800 * 0.034 ~ 27 N.
GRIPPER_GAINS = PdControllerGains(p=800.0, d=10.0)

GRIPPER_OPEN = np.array([-scene.GRIPPER_OPEN, scene.GRIPPER_OPEN])
GRIPPER_CLOSED = np.array([0.0, 0.0])


# ---------------------------------------------------------------- trajectories
def hold(q, duration) -> PiecewisePolynomial:
    q = np.asarray(q, float).reshape(-1, 1)
    return PiecewisePolynomial.ZeroOrderHold([0.0, duration], np.hstack([q, q]))


def smooth_samples(samples: Sequence[np.ndarray], dt: float) -> PiecewisePolynomial:
    """Dense samples (e.g. from diff IK) -> C2 cubic spline, zero end velocity."""
    Q = np.array(samples).T
    t = np.arange(Q.shape[1]) * dt
    z = np.zeros((Q.shape[0], 1))
    return PiecewisePolynomial.CubicWithContinuousSecondDerivatives(t, Q, z, z)


def ramp(q0, q1, duration) -> PiecewisePolynomial:
    Q = np.column_stack([q0, q1])
    return PiecewisePolynomial.CubicHermite([0.0, duration], Q, np.zeros_like(Q))


def concat(segments: Sequence[PiecewisePolynomial]) -> PiecewisePolynomial:
    out = None
    for seg in segments:
        seg = copy.copy(seg)
        if out is None:
            seg.shiftRight(-seg.start_time())
            out = seg
        else:
            seg.shiftRight(out.end_time() - seg.start_time())
            out.ConcatenateInTime(seg)
    return out


# ---------------------------------------------------------------- diagram
def build_sim(meshcat, base_traj, arm_traj, gripper_traj, time_step=0.001, phone_pose=None):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        # "similar" (a SAP-family solver, so joint PD control still works):
        # plain "sap" let the light phone settle ~8 mm into the table even
        # with an explicit point-contact stiffness. See D-007.
        MultibodyPlantConfig(time_step=time_step, discrete_contact_approximation="similar"),
        builder,
    )
    scene.load_scene(Parser(plant), phone_pose=phone_pose)
    base = plant.GetModelInstanceByName(scene.BASE_INSTANCE)
    arm = plant.GetModelInstanceByName(scene.ARM_INSTANCE)
    wsg = plant.GetModelInstanceByName(scene.GRIPPER_INSTANCE)
    phone = plant.GetModelInstanceByName("phone")

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

    for inst, traj in [(base, base_traj), (arm, arm_traj), (wsg, gripper_traj)]:
        src = builder.AddSystem(TrajectorySource(traj, output_derivative_order=1))
        builder.Connect(src.get_output_port(), plant.get_desired_state_input_port(inst))

    loggers = {}
    for name, inst in [("base", base), ("arm", arm), ("phone", phone), ("wsg", wsg)]:
        log = builder.AddSystem(VectorLogSink(plant.num_multibody_states(inst)))
        builder.Connect(plant.get_state_output_port(inst), log.get_input_port())
        loggers[name] = log

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    return builder.Build(), plant, visualizer, loggers


def draw_polyline(meshcat, path, name, z, width, rgba: Rgba):
    pts = np.array([[q[0], q[1], z] for q in path]).T
    meshcat.SetLine(name, pts, width, rgba)
