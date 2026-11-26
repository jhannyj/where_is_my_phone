"""
Persistent world-frame voxel map from a Drake depth camera,
plus a simple robot-vs-voxel collision check.
"""
import numpy as np
from scipy.spatial import cKDTree # helps calculate distance from voxels to robot spheres

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

ROBOT_URL = "where_is_my_phone/models/mobile_manipulator.dmd.yaml"
#CAMERA_BODY = ""
#X_BODY_CAM = RigidTransform(RollPitchYaw(0, 0, 0), [0, 0, 0.1])  # TODO: mounting pose
ROOM_SDF = "where_is_my_phone/models/living_room/living_room_scene.dmd.yaml"

parser = Parser(plant)
ProcessModelDirectives(LoadModelDirectives("mobile_manipulator.dmd.yaml"), plant, parser)
plant.Finalize()

cam_body = plant.GetBodyByName("camera_body", plant.GetModelInstanceByName("zed_camera"))

# Rotation from Drake's camera convention to the ZED body frame (see below)
X_BC = RigidTransform(RotationMatrix(np.array([[0, 0, 1],
                                               [-1, 0, 0],
                                               [0, -1, 0]])))

sensor = builder.AddSystem(RgbdSensor(plant.GetBodyFrameIdOrThrow(cam_body.index()), X_BC, color_cam, depth_cam))



# get collision pose for each robot link in mobile manipulator
def robot_collision_shapes(plant, plant_ctx, robot, scene_graph):
    inspector = scene_graph.model_inspector()
    out = []
    for bi in plant.GetBodyIndices(robot):
        body = plant.get_body(bi)
        X_WB = plant.EvalBodyPoseInWorld(plant_ctx, body)
        for gid in plant.GetCollisionGeometriesForBody(body):
            X_WG = X_WB @ inspector.GetPoseInFrame(gid)
            out.append((inspector.GetShape(gid), X_WG))
    return out


class VoxelMap:
    def __init__(self, voxel_size):
        self.s = voxel_size
        self.hits = {}          # (i, j, k) -> observation count
        self._tree = None
        self._dirty = True

    def integrate(self, pts_W):
        """pts_W: Nx3 world-frame points"""
        if len(pts_W) == 0:
            return
        idx = np.unique(np.floor(pts_W / self.s).astype(int), axis=0)
        for key in map(tuple, idx):
            self.hits[key] = self.hits.get(key, 0) + 1
        self._dirty = True

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