"""
Persistent world-frame voxel map from a Drake depth camera,
plus a simple robot-vs-voxel collision check.
"""
import numpy as np
from scipy.spatial import cKDTree

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
    PointCloud,
    RenderCameraCore,
    RenderEngineVtkParams,
    RgbdSensor,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)

VOXEL_SIZE = 0.03          # 3 cm voxels
MIN_HITS = 2               # observations before a voxel counts as occupied
MAP_PERIOD = 0.1           # s, how often the map is updated
ROBOT_SPHERE_R = 0.08      # m, rough link thickness (used for masking + collision)
SELF_MASK_MARGIN = 0.05    # m, extra margin when removing robot's own points
IMG_W, IMG_H, FOV_Y = 320, 240, np.pi / 3
MAX_RANGE = 5.0

ROBOT_URL = ""
CAMERA_BODY = ""
X_BODY_CAM = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, 0.1])  # TODO: mounting pose
ROOM_SDF = ""


# ----------------------------------------------------------------------------
# Voxel map (lives in the WORLD frame, persists across frames)
# ----------------------------------------------------------------------------
class VoxelMap:
    def __init__(self, voxel_size):
        self.s = voxel_size
        self.hits = {}          # (i, j, k) -> observation count
        self._tree = None
        self._dirty = True

    def integrate(self, pts_W):
        """pts_W: Nx3 world-frame points (already filtered)."""
        if len(pts_W) == 0:
            return
        idx = np.unique(np.floor(pts_W / self.s).astype(int), axis=0)
        for key in map(tuple, idx):
            self.hits[key] = self.hits.get(key, 0) + 1
        self._dirty = True
        # TODO (optional): free-space clearing by raycasting from the camera
        # origin to each hit and decrementing/removing voxels along the ray.

    def occupied_centers(self, min_hits=MIN_HITS):
        keys = [k for k, n in self.hits.items() if n >= min_hits]
        if not keys:
            return np.empty((0, 3))
        return (np.array(keys) + 0.5) * self.s

    def tree(self):
        if self._dirty:
            c = self.occupied_centers()
            self._tree = cKDTree(c) if len(c) else None
            self._dirty = False
        return self._tree

def robot_spheres_W(body_poses, robot_body_indices, per_link=4):
    """Return Mx3 sphere centers (world) interpolated between link origins.
    TODO: replace with something tighter (e.g. per-link collision geometry)."""
    origins = np.array([body_poses[i].translation() for i in robot_body_indices])
    pts = [origins[0]]
    for a, b in zip(origins[:-1], origins[1:]):
        for t in np.linspace(0, 1, per_link + 1)[1:]:
            pts.append((1 - t) * a + t * b)
    return np.array(pts)


def in_collision(voxel_map, spheres_W, radius=ROBOT_SPHERE_R):
    tree = voxel_map.tree()
    if tree is None:
        return False
    d, _ = tree.query(spheres_W)
    return bool(np.any(d < radius + 0.5 * voxel_map.s))

class VoxelMapper(LeafSystem):
    def __init__(self, voxel_map, robot_body_indices, period=MAP_PERIOD):
        super().__init__()
        self.map = voxel_map
        self.robot_bodies = robot_body_indices
        self.cloud_port = self.DeclareAbstractInputPort(
            "point_cloud", AbstractValue.Make(PointCloud()))
        self.poses_port = self.DeclareAbstractInputPort(
            "body_poses", AbstractValue.Make([RigidTransform()]))
        self.DeclarePeriodicPublishEvent(period, 0.0, self._update)

    def _update(self, context):
        cloud = self.cloud_port.Eval(context)
        poses = self.poses_port.Eval(context)

        pts = np.asarray(cloud.xyzs()).T                      # Nx3, world frame
        pts = pts[np.isfinite(pts).all(axis=1)]               # drop invalid pixels
        if len(pts) == 0:
            return

        # Drop the robot's own points so the arm doesn't map itself.
        spheres = robot_spheres_W(poses, self.robot_bodies)
        d, _ = cKDTree(spheres).query(pts)
        pts = pts[d > ROBOT_SPHERE_R + SELF_MASK_MARGIN]

        # TODO: exclude the grasp target (phone) (if necessary)
        self.map.integrate(pts)


def build():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    parser = Parser(plant)

    robot = parser.AddModels(ROBOT_URL)[0]
    parser.AddModelsFromString(ROOM_SDF, "sdf")
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("iiwa_link_0", robot))  # TODO: base frame name
    plant.Finalize()

    robot_body_indices = list(plant.GetBodyIndices(robot))

    # Renderer + depth camera mounted on a robot body (moves with the robot).
    scene_graph.AddRenderer("renderer", MakeRenderEngineVtk(RenderEngineVtkParams()))
    info = CameraInfo(IMG_W, IMG_H, FOV_Y)
    core = RenderCameraCore("renderer", info, ClippingRange(0.01, MAX_RANGE + 1),
                            RigidTransform())
    color_cam = ColorRenderCamera(core, show_window=False)
    depth_cam = DepthRenderCamera(core, DepthRange(0.1, MAX_RANGE))

    cam_body = plant.GetBodyByName(CAMERA_BODY, robot)
    sensor = builder.AddSystem(RgbdSensor(
        plant.GetBodyFrameIdOrThrow(cam_body.index()), X_BODY_CAM,
        color_cam, depth_cam))
    builder.Connect(scene_graph.get_query_output_port(),
                    sensor.query_object_input_port())

    # Depth -> world-frame point cloud. camera_pose input is X_PC with P = world.
    d2pc = builder.AddSystem(DepthImageToPointCloud(info, fields=BaseField.kXYZs))
    builder.Connect(sensor.depth_image_32F_output_port(),
                    d2pc.depth_image_input_port())
    builder.Connect(sensor.body_pose_in_world_output_port(),
                    d2pc.GetInputPort("camera_pose"))

    voxel_map = VoxelMap(VOXEL_SIZE)
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

    n = plant.num_positions(robot)
    t = 0.0
    for q1 in np.linspace(-1.0, 1.0, 40):        # TODO: replace with your planner/controller
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
    spheres = robot_spheres_W(poses, robot_bodies)
    print("q_test in collision:", in_collision(voxel_map, spheres))


if __name__ == "__main__":
    main()