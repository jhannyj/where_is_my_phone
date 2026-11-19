"""Cheap pre-filter for grasp candidates: is the gripper alone, floating at
the grasp (and pre-grasp) pose with fingers open, collision-free?

This is the standard step before IK in the course's antipodal pipeline. It
rejects candidates whose fingertips would hit the table without paying for
an IK solve. It needs only a model of the environment, so the same check
works later against the voxel map.
"""

import os

import numpy as np
from pydrake.all import (
    LoadModelDirectives,
    ProcessModelDirectives,
    RigidTransform,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)

from wimp import scene
from wimp.planning.planning_scene import GRIPPER_ENV_PADDING

WSG_URL = "package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf"


class GripperChecker:
    def __init__(self, phone_pose: RigidTransform = None):
        builder = RobotDiagramBuilder(time_step=0.0)
        parser = builder.parser()
        scene.add_packages(parser)
        room = os.path.join(scene.REPO_ROOT, "living_room_scene", "living_room_scene.dmd.yaml")
        ProcessModelDirectives(LoadModelDirectives(room), builder.plant(), parser)
        plant = builder.plant()
        (wsg,) = parser.AddModels(url=WSG_URL)
        if phone_pose is not None:
            plant.SetDefaultFloatingBaseBodyPose(plant.GetBodyByName("phone_link"), phone_pose)
        for j in plant.GetJointIndices(wsg):
            joint = plant.get_joint(j)
            if joint.num_positions() == 1:
                sign = -1.0 if "left" in joint.name() else 1.0
                joint.set_default_positions([sign * scene.GRIPPER_OPEN])
        self.diagram = builder.Build()
        self.plant = plant
        self.body = plant.GetBodyByName("body", wsg)
        self.checker = SceneGraphCollisionChecker(
            model=self.diagram,
            robot_model_instances=[wsg],
            edge_step_size=0.01,
            env_collision_padding=GRIPPER_ENV_PADDING,
            self_collision_padding=0.0,
        )
        self.context = plant.CreateDefaultContext()

    def is_free(self, X_WG: RigidTransform) -> bool:
        self.plant.SetFreeBodyPose(self.context, self.body, X_WG)
        return bool(self.checker.CheckConfigCollisionFree(self.plant.GetPositions(self.context)))

    def filter(self, grasps, standoff=0.10):
        return [g for g in grasps if self.is_free(g.X_WG) and self.is_free(g.pregrasp(standoff))]
