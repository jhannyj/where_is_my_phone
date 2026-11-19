"""The planner's model of the world: a Drake RobotDiagram of the scene plus a
collision checker. Base planning, arm planning, IK and base placement all
query the same object.

Assumes the scene is known (privileged information). Later, the environment
part will come from the voxel map instead.
"""

from typing import Optional

import numpy as np
from pydrake.all import (
    BodyIndex,
    RigidTransform,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)

from wimp import scene

# D-003: clearance (m) the robot keeps from the environment while moving.
ENV_PADDING = 0.03
# The gripper must be allowed close to the phone and the table to grasp,
# so its links get a much smaller margin.
GRIPPER_ENV_PADDING = 0.002
EDGE_RESOLUTION = 0.05


class PlanningScene:
    def __init__(self, scene_file: str = scene.SCENE_FILE, phone_pose=None):
        builder = RobotDiagramBuilder(time_step=0.0)
        scene.load_scene(builder.parser(), scene_file, phone_pose=phone_pose)
        self.diagram = builder.Build()
        self.plant = self.diagram.plant()
        self.idx = scene.robot_indices(self.plant)

        robot_instances = scene.robot_instances(self.plant)
        self.checker = SceneGraphCollisionChecker(
            model=self.diagram,
            robot_model_instances=robot_instances,
            edge_step_size=EDGE_RESOLUTION,
            env_collision_padding=ENV_PADDING,
            self_collision_padding=0.0,
        )
        robot_bodies = {
            b for inst in robot_instances for b in self.plant.GetBodyIndices(inst)
        }
        env_bodies = [
            BodyIndex(i) for i in range(self.plant.num_bodies())
            if BodyIndex(i) not in robot_bodies
        ]
        wsg = self.plant.GetModelInstanceByName(scene.GRIPPER_INSTANCE)
        for g in self.plant.GetBodyIndices(wsg):
            for e in env_bodies:
                self.checker.SetPaddingBetween(g, e, GRIPPER_ENV_PADDING)

        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.context)
        # Template configuration: robot at its defaults (arm stowed, gripper
        # open), phone and everything else at their default poses.
        self.q_template = self.plant.GetPositions(self.plant_context).copy()
        self.gripper_frame = self.plant.GetBodyByName("body", wsg).body_frame()

    # --- configuration helpers ---------------------------------------------
    def full_q(self, q_base=None, q_arm=None) -> np.ndarray:
        q = self.q_template.copy()
        if q_base is not None:
            q[self.idx.base_q] = q_base
        if q_arm is not None:
            q[self.idx.arm_q] = q_arm
        return q

    def is_free(self, q_full) -> bool:
        return bool(self.checker.CheckConfigCollisionFree(q_full))

    def all_free(self, qs) -> bool:
        return all(self.checker.CheckConfigsCollisionFree(list(qs)))

    def default_base_q(self):
        return self.q_template[self.idx.base_q].copy()

    def arm_stow(self):
        return self.q_template[self.idx.arm_q].copy()

    def arm_limits(self):
        lo = self.plant.GetPositionLowerLimits()[self.idx.arm_q]
        hi = self.plant.GetPositionUpperLimits()[self.idx.arm_q]
        return lo, hi

    def gripper_pose(self, q_full) -> RigidTransform:
        self.plant.SetPositions(self.plant_context, q_full)
        return self.plant.CalcRelativeTransform(
            self.plant_context, self.plant.world_frame(), self.gripper_frame
        )

    def phone_pose(self, q_full: Optional[np.ndarray] = None) -> RigidTransform:
        self.plant.SetPositions(self.plant_context, self.q_template if q_full is None else q_full)
        return self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.plant.GetBodyByName("phone_link")
        )
