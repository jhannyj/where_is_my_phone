"""Build and exercise a persistent world-frame voxel map from a Drake depth camera."""

import numpy as np

from pydrake.all import (
    AbstractValue,
    AddMultibodyPlantSceneGraph,
    BaseField,
    CameraInfo,
    ClippingRange,
    ColorRenderCamera,
    DepthImageToPointCloud,
    DepthRange,
    DepthRenderCamera,
    DiagramBuilder,
    LeafSystem,
    MakeRenderEngineVtk,
    Parser,
    RenderCameraCore,
    RenderEngineVtkParams,
    RgbdSensor,
    RigidTransform,
    RotationMatrix,
    Simulator,
)

from wimp import scene
from wimp.voxel_map_functions import (
    MAP_PERIOD,
    VOXEL_SIZE,
    VoxelMap,
    VoxelMapper,
    in_collision,
    robot_collision_geometries,
    robot_geometry_poses,
)

IMG_W, IMG_H, FOV_Y = 320, 240, np.pi / 3
MAX_RANGE = 5.0

# Camera optical frame relative to the camera body frame.
X_BODY_CAM = RigidTransform(RotationMatrix(np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
])))


class _CameraPoseInWorld(LeafSystem):
    """Compose camera body's world pose with its body-to-camera extrinsic."""

    def __init__(self, X_BODY_CAM):
        super().__init__()
        self.X_BODY_CAM = X_BODY_CAM
        self.body_pose_input = self.DeclareAbstractInputPort(
            "body_pose", AbstractValue.Make(RigidTransform()))
        self.DeclareAbstractOutputPort(
            "camera_pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self._calc_camera_pose,
        )

    def _calc_camera_pose(self, context, output):
        output.set_value(self.body_pose_input.Eval(context) @ self.X_BODY_CAM)


def build():
    """Build room + robot + RGB-D sensor and connect the voxel mapper."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    parser = Parser(plant)
    scene.load_scene(parser)
    plant.Finalize()

    scene_graph.AddRenderer(
        "renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
    camera_instance = plant.GetModelInstanceByName(scene.CAMERA_INSTANCE)
    camera_body = plant.GetBodyByName("camera_body", camera_instance)

    info = CameraInfo(width=IMG_W, height=IMG_H, fov_y=FOV_Y)
    core = RenderCameraCore(
        info, ClippingRange(0.1, MAX_RANGE + 1), RigidTransform())
    color_camera = ColorRenderCamera(core, show_window=True)
    depth_camera = DepthRenderCamera(core, DepthRange(0.1, MAX_RANGE))
    camera_sensor = builder.AddSystem(RgbdSensor(
        plant.GetBodyFrameIdOrThrow(camera_body.index()), X_BODY_CAM,
        color_camera, depth_camera))
    builder.Connect(scene_graph.get_query_output_port(),
                    camera_sensor.query_object_input_port())

    depth_to_cloud = builder.AddSystem(
        DepthImageToPointCloud(info, fields=BaseField.kXYZs))
    builder.Connect(camera_sensor.depth_image_32F_output_port(),
                    depth_to_cloud.depth_image_input_port())
    camera_pose = builder.AddSystem(_CameraPoseInWorld(X_BODY_CAM))
    builder.Connect(camera_sensor.body_pose_in_world_output_port(),
                    camera_pose.GetInputPort("body_pose"))
    builder.Connect(camera_pose.GetOutputPort("camera_pose"),
                    depth_to_cloud.GetInputPort("camera_pose"))

    robot_bodies = [
        body_index
        for instance in scene.robot_instances(plant)
        for body_index in plant.GetBodyIndices(instance)
    ]
    collision_geometries = robot_collision_geometries(
        plant, robot_bodies, scene_graph)
    voxel_map = VoxelMap(VOXEL_SIZE)
    mapper = builder.AddSystem(VoxelMapper(voxel_map, collision_geometries))
    builder.Connect(depth_to_cloud.GetOutputPort("point_cloud"),
                    mapper.GetInputPort("point_cloud"))
    builder.Connect(plant.get_body_poses_output_port(),
                    mapper.GetInputPort("body_poses"))

    diagram = builder.Build()
    return diagram, plant, scene_graph, voxel_map, robot_bodies


def main():
    diagram, plant, scene_graph, voxel_map, robot_bodies = build()
    simulator = Simulator(diagram)
    root_context = simulator.get_mutable_context()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    indices = scene.robot_indices(plant)

    # Move the base through a short sequence while the camera integrates scans.
    q = plant.GetPositions(plant_context).copy()
    t = 0.0
    for x in np.linspace(-3.5, -2.5, 40):
        q[indices.base_q[0]] = x
        q[indices.base_q[1]] = 0.5
        plant.SetPositions(plant_context, q)
        t += MAP_PERIOD
        simulator.AdvanceTo(t)
        print(f"t={t:.1f} occupied voxels: {len(voxel_map.occupied_centers())}")

    # Evaluate the real registered robot collision shapes at another base pose.
    q[indices.base_q[0]] = -2.0
    plant.SetPositions(plant_context, q)
    simulator.AdvanceTo(t + MAP_PERIOD)
    body_poses = plant.get_body_poses_output_port().Eval(plant_context)
    geometry_poses_W = robot_geometry_poses(
        body_poses,
        robot_collision_geometries(plant, robot_bodies, scene_graph),
    )
    print("candidate configuration in collision:",
          in_collision(voxel_map, geometry_poses_W))


if __name__ == "__main__":
    main()
