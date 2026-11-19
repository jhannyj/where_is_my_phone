"""Scene construction shared by planning and simulation.

Which models are in the room lives in models/nav_scene.dmd.yaml (living room
+ phone) and models/mobile_manipulator.dmd.yaml (the robot). This module loads
them and sets the robot's start pose and stow posture.
"""

import os
from dataclasses import dataclass

import numpy as np
from pydrake.all import (
    LoadModelDirectives,
    Meshcat,
    MeshcatParams,
    Parser,
    ProcessModelDirectives,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_FILE = os.path.join(REPO_ROOT, "models", "nav_scene.dmd.yaml")

BASE_INSTANCE = "mobile_base"
ARM_INSTANCE = "iiwa"
GRIPPER_INSTANCE = "wsg"
CAMERA_INSTANCE = "zed_camera"
# Everything rigidly attached to the robot; the collision checker treats
# these as "robot" and everything else as environment.
ROBOT_INSTANCES = (BASE_INSTANCE, ARM_INSTANCE, GRIPPER_INSTANCE, CAMERA_INSTANCE)
BASE_JOINTS = ("x_joint", "y_joint", "yaw_joint")

# Default start pose of the base (x, y, yaw) and the folded arm posture used
# while driving (D-004).
DEFAULT_BASE_START = (-3.5, 0.0, 0.0)
ARM_STOW = (0.0, -0.3, 0.0, -2.0, 0.0, 1.0, 0.0)
GRIPPER_OPEN = 0.05  # m, each WSG finger joint (fully open is ~0.055)

# Room interior is x, y in [-5, 5]; planners sample inside the walls.
ROOM_HALF_EXTENT = 5.0


def add_packages(parser: Parser) -> None:
    pm = parser.package_map()
    pm.Add("living_room", os.path.join(REPO_ROOT, "living_room_scene"))
    pm.Add("custom_models", os.path.join(REPO_ROOT, "models"))


def load_scene(parser: Parser, scene_file: str = SCENE_FILE, base_start=DEFAULT_BASE_START):
    """Add all scene models to parser's plant and set the robot's default
    configuration. The plant is NOT finalized."""
    add_packages(parser)
    plant = parser.plant()
    models = ProcessModelDirectives(LoadModelDirectives(scene_file), plant, parser)

    base = plant.GetModelInstanceByName(BASE_INSTANCE)
    for name, value in zip(BASE_JOINTS, base_start):
        plant.GetJointByName(name, base).set_default_positions([value])
    for joint, value in zip(arm_joints(plant), ARM_STOW):
        joint.set_default_positions([value])
    for joint in gripper_joints(plant):
        sign = -1.0 if "left" in joint.name() else 1.0
        joint.set_default_positions([sign * GRIPPER_OPEN])
    return models


def _one_dof_joints(plant, instance_name):
    instance = plant.GetModelInstanceByName(instance_name)
    joints = [plant.get_joint(j) for j in plant.GetJointIndices(instance)]
    return [j for j in joints if j.num_positions() == 1]


def arm_joints(plant):
    return _one_dof_joints(plant, ARM_INSTANCE)


def gripper_joints(plant):
    return _one_dof_joints(plant, GRIPPER_INSTANCE)


def robot_instances(plant):
    return [plant.GetModelInstanceByName(n) for n in ROBOT_INSTANCES]


@dataclass
class RobotIndices:
    """Where robot joints live inside the full plant position vector."""

    base_q: np.ndarray  # (x, y, yaw)
    arm_q: np.ndarray  # 7 iiwa joints


def robot_indices(plant) -> RobotIndices:
    base = plant.GetModelInstanceByName(BASE_INSTANCE)
    return RobotIndices(
        base_q=np.array([plant.GetJointByName(j, base).position_start() for j in BASE_JOINTS]),
        arm_q=np.array([j.position_start() for j in arm_joints(plant)]),
    )


def start_meshcat() -> Meshcat:
    """Meshcat that also works from inside Docker (MESHCAT_HOST='*')."""
    params = MeshcatParams()
    params.host = os.environ.get("MESHCAT_HOST", "localhost")
    if "MESHCAT_PORT" in os.environ:
        params.port = int(os.environ["MESHCAT_PORT"])
    meshcat = Meshcat(params)
    print(f"Meshcat: {meshcat.web_url().replace('*', 'localhost')}")
    return meshcat
