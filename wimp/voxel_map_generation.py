"""
Persistent world-frame voxel map from a Drake depth camera,
plus a simple robot-vs-voxel collision check.
"""
from html import parser

import numpy as np
from scipy.spatial import cKDTree # helps calculate distance from voxels to robot spheres
from voxel_map_functions import *

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
    LoadModelDirectives,
    MakeRenderEngineVtk,
    Parser,
    PointCloud,
    ProcessModelDirectives,
    RenderCameraCore,
    RenderEngineVtkParams,
    RgbdSensor,
    RigidTransform,
    RotationMatrix,
    RollPitchYaw,
    Simulator,
)

VOXEL_SIZE = 0.03          # 3 cm voxels

MIN_HITS = 2               # observations before a voxel counts as occupied

MAP_PERIOD = 0.1           # s, how often the map is updated

ROBOT_SPHERE_R = 0.08      # m, rough link thickness (used for masking + collision)

SELF_MASK_MARGIN = 0.05    # m, extra margin when removing robot's own points

IMG_W, IMG_H, FOV_Y = 320, 240, np.pi / 3 # image width, height, and vertical field of view (radians)

MAX_RANGE = 5.0 # 5 m max range perception---good for living room scenario


ROBOT_URL = "where_is_my_phone/models/mobile_manipulator.dmd.yaml"
X_BODY_CAM = RigidTransform(RotationMatrix(np.array([[0, 0, 1],
                                               [-1, 0, 0],
                                               [0, -1, 0]])))
ROOM_SDF = "where_is_my_phone/models/living_room/living_room_scene.dmd.yaml"

def build():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    parser = Parser(plant)

    ProcessModelDirectives(LoadModelDirectives(ROBOT_URL), plant, parser)
    robot = parser.AddModels(ROBOT_URL)    
    parser.AddModelsFromString(ROOM_SDF, "sdf")

    CAMERA_BODY = plant.GetBodyByName("camera_body", plant.GetModelInstanceByName("zed_camera"))

    # scene finalization
    plant.Finalize()

    # Initializng the camera parameters
    cam_body = plant.GetBodyByName(CAMERA_BODY, robot)
    scene_graph.AddRenderer("renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
    info = CameraInfo(width=IMG_W, height=IMG_H, fov_y=FOV_Y)
    core = RenderCameraCore(info, ClippingRange(0.1, MAX_RANGE + 1), RigidTransform()) # use identity matrix for the camera pose in the camera frame
    color_cam = ColorRenderCamera(core, show_window=True) # show_window=True to visualize the camera view
    depth_cam = DepthRenderCamera(core, DepthRange(0.1, MAX_RANGE))

    camera_sensor = builder.AddSystem(RgbdSensor(
        plant.GetBodyFrameIdOrThrow(cam_body.index()), X_BODY_CAM,
        color_cam, depth_cam))
    builder.Connect(scene_graph.get_query_output_port(),
                    camera_sensor.query_object_input_port())

    # Depth -> world-frame point cloud. camera_pose input is X_PC with P = world.
    d2pc = builder.AddSystem(DepthImageToPointCloud(info, fields=BaseField.kXYZs))
    builder.Connect(camera_sensor.depth_image_32F_output_port(),
                    d2pc.depth_image_input_port())
    builder.Connect(camera_sensor.body_pose_in_world_output_port(),
                    d2pc.GetInputPort("camera_pose"))

    voxel_map = VoxelMap(VOXEL_SIZE)
    robot_body_indices = [plant.GetBodyIndices(robot)[i] for i in range(plant.num_bodies(robot))]

    mapper = builder.AddSystem(VoxelMapper(voxel_map, robot_body_indices))
    builder.Connect(d2pc.GetOutputPort("point_cloud"), mapper.GetInputPort("point_cloud"))
    builder.Connect(plant.get_body_poses_output_port(), mapper.GetInputPort("body_poses"))

    diagram = builder.Build()
    return diagram, plant, voxel_map, robot, robot_body_indices

 
def main():
    diagram, plant, voxel_map, robot, robot_bodies = build()
    simulator = Simulator(diagram)
    root_ctx = simulator.get_mutable_context()
    plant_ctx = plant.GetMyMutableContextFromRoot(root_ctx)

    # test the robot's collision shapes at a few configurations, and print the number of occupied voxels
    n = plant.num_positions(robot)
    t = 0.0
    for q1 in np.linspace(-1.0, 1.0, 40):       
        q = np.zeros(n)
        q[0] = q1
        q[1] = 0.5
        # No controller here, so we just teleport the joints each step.
        plant.SetPositions(plant_ctx, robot, q)
        plant.SetVelocities(plant_ctx, robot, np.zeros(plant.num_velocities(robot)))
        t += MAP_PERIOD
        simulator.AdvanceTo(t)
        print(f"t={t:.1f}  occupied voxels: {len(voxel_map.occupied_centers())}")

    # Collision check for a candidate configuration.
    q_test = np.zeros(n)
    q_test[1] = 1.2
    plant.SetPositions(plant_ctx, robot, q_test)
    poses = plant.get_body_poses_output_port().Eval(plant_ctx)
    robot_collision_shapes = robot_collision_shapes(plant, plant_ctx, robot, diagram.GetSubsystemByName("scene_graph"))
    print("q_test in collision:", in_collision(voxel_map, spheres))


if __name__ == "__main__":
    main()