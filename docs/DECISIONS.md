# Robotics design decisions

Every robot-level algorithm choice goes here: the options we considered, what
we picked, why, and what would make us change our minds. We use this file for
the report and for answering staff questions. (See CLAUDE.md, "How we work".)

**Status key:** ✅ decided by the team · 🟡 default proposed during coding, still
needs team sign-off · ⏳ open, to decide later

---

## D-001 Robot model ✅

*2026-11-17: stand-in chosen by Jhanny. 2026-11-18: replaced by the team's robot.*

**Context:** We needed a robot to start base and arm work before the real
model existed.

| Option | Pros | Cons |
|---|---|---|
| **Holonomic base: three actuated virtual joints (x, y, yaw) carrying a box chassis, with the arm on top** | Simple; lets us plan in (x, y, yaw) directly; the wheels don't need simulating | Ignores wheel kinematics |
| Differential-drive base | More realistic | The planner must respect nonholonomic constraints: much more work, and not the focus of the project |

**Decision:** holonomic base. This is now the team's real robot,
`models/mobile_manipulator.dmd.yaml`: a 0.8 × 0.6 × 0.2 m chassis
(`mobile_base.sdf`, wheels are visual only), plus the iiwa (D-009), a WSG 50
gripper, and a wrist-mounted ZED camera. On 2026-11-18 I (Claude) moved the
x/y/yaw joints inside `<model>` (they had been written after `</sdf>` in URDF
syntax, so Drake ignored them). I also made yaw a continuous joint so the base
can turn either way, and raised the chassis to axle height so the wheels sit
on the floor.

**Revisit if:** we want a nonholonomic base. The planner and controller would
then need to change (see D-002).

---

## D-002 Base path planner (known map) ✅

*Date: 2026-11-17 · Decided by: Jhanny*

**Context:** Milestone 1. The robot knows the whole scene (privileged
information) and must drive from its start pose to a goal pose.

| Option | Pros | Cons |
|---|---|---|
| **RRT-Connect in (x, y, yaw)** | Probabilistically complete; collision-checks the robot's real 3D shape (so arm overhang and rotation matter); fast in open rooms because the two trees greedily connect; it is the planner in our proposal | Paths are jagged (needs smoothing); not optimal; random, so results vary run to run |
| A* on a 2D occupancy grid | Resolution-complete; optimal at grid resolution; deterministic; fits the voxel map we will build later (project it to 2D) | Treats the robot as a disk (inflated obstacles), so it ignores orientation unless we add a yaw dimension; results depend on grid resolution |
| PRM | Reusable roadmap for many queries | Wasted effort for one query per run |
| GCS / trajectory optimization | Smooth, near-optimal | Needs a convex decomposition of free space; heavy for this milestone |

**Decision:** RRT-Connect (Kuffner & LaValle 2000), implemented by us in
`wimp/planning/rrt_connect.py` (no Drake dependency, unit-tested in
`tests/test_rrt_connect.py`), followed by shortcutting (D-005).

**Revisit if:** we move to the unknown-map setting (see OPEN-1).

---

## D-003 Collision model for base planning 🟡

| Option | Pros | Cons |
|---|---|---|
| **Full 3D geometry (base box + iiwa collision primitives) vs. scene, through Drake's `SceneGraphCollisionChecker`, with 3 cm padding** | Exact for our models; accounts for arm overhang and base rotation; same checker can later check arm motions | Slower per check than a 2D grid lookup |
| 2D disk footprint vs. inflated 2D obstacles | Very fast; standard for mobile robots | Conservative (the circle must cover the square base and the arm); ignores the couch overhang (the robot can't reach under it anyway) |

**Default chosen:** full 3D with `ENV_PADDING = 0.03` m (a safety margin, so
small controller tracking errors don't cause contact). Edges are checked every
5 cm in the weighted metric (D-004).

---

## D-004 Configuration space and distance metric 🟡

| Option | Pros | Cons |
|---|---|---|
| **Plan only (x, y, yaw); arm held in a fixed "stow" posture** | 3-D search: fast; driving with a folded arm is what real mobile manipulators do | Can't use arm motion to squeeze through tight spots |
| Plan all 10 DoF (base + arm) at once | Most general | 10-D search: much slower; produces strange arm motions while driving |

Distance: `sqrt(dx² + dy² + (w · dyaw)²)` with `w = 0.5` m/rad. Yaw differences
wrap to the short way round. The weight w ≈ base half-diagonal (0.42 m),
i.e. roughly how far the robot's edge sweeps per radian of turning, which puts
translation and rotation on a comparable scale.

**Default chosen:** (x, y, yaw) with the stowed arm, w = 0.5.

---

## D-005 Path post-processing 🟡

| Option | Pros | Cons |
|---|---|---|
| **Random shortcutting (200 tries)** | Simple; never makes the path longer; each new segment is collision-checked | Still piecewise linear with corners |
| None | Simplest | Jagged RRT paths, long and wiggly |
| Spline / trajectory-optimization smoothing | Smooth | Smoothed curves leave the checked segments, so they must be re-checked; more complex |

**Default chosen:** random shortcutting.

---

## D-006 Time parameterization 🟡

| Option | Pros | Cons |
|---|---|---|
| **Stop at each waypoint; cubic timing `s = 3τ² − 2τ³` per straight segment** | The robot stays exactly on the collision-checked straight segments; velocity is continuous; respects v_max = 0.5 m/s and ω_max = 0.8 rad/s | Slow: stops at every corner |
| Constant speed per segment (linear interpolation) | Simple, faster | Velocity jumps at corners, causing tracking overshoot that could cut corners |
| TOPP-RA (time-optimal with acceleration limits) | Fastest feasible timing | More machinery; a spline through the waypoints leaves the checked segments |

**Default chosen:** stop-at-waypoint cubic timing (`time_parameterize` in
`wimp/planning/base_planner.py`). We value safety over speed for now.
Time-to-grasp is a metric, but it is compared between methods that share this
same timing, so the comparison stays fair.

---

## D-007 Low-level control (placeholder) 🟡

| Option | Pros | Cons |
|---|---|---|
| **Joint-level PD on all 10 joints (Drake's built-in actuator PD) + ideal gravity compensation (gravity off for the robot)** | Few lines of code; physics and contact are still simulated, so collisions would show up | Gravity compensation is idealized |
| `InverseDynamicsController` (as in the course's HardwareStation) | Realistic computed-torque control | Needs a separate control plant; more wiring for a stand-in |
| Purely kinematic playback (set positions, no physics) | Trivial | No contact physics, so collisions can't be detected; we will need physics for grasping anyway |

**Default chosen:** PD + ideal gravity compensation. Expected to be replaced
by the real robot's controller.

---

## D-008 Goal base pose 🟡

**Context:** For milestone 1 the goal pose is hand-picked: 0.8 m west of the
phone, facing it. Choosing a base pose from which the arm can actually reach
and grasp the phone is a real algorithm decision for later (OPEN-2).

---

## D-009 Arm model ✅

*2026-11-18 · Decided by: Jhanny*

The team's first robot file used `iiwa7_no_collision`, which has no collision
geometry, so nothing could check the arm against furniture.

| Option | Pros | Cons |
|---|---|---|
| iiwa 7 with box collision | Smallest change from the teammates' file; lighter | Differs from the proposal |
| **iiwa 14 with primitive collision** | Matches the proposal; primitive (cylinder/sphere) collision shapes are cheap to check | Heavier arm |

**Decision:** iiwa 14,
`drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf`.

---

## D-010 Robot localization ✅

*2026-11-18 · Decided by: Jhanny*

| Option | Pros | Cons |
|---|---|---|
| **Ground-truth robot pose** | No extra work; keeps the experiment about *search efficiency*: both methods get the same assumption, so the vision-vs-audio comparison is not confounded by localization error | Less realistic |
| Ground truth + simulated odometry drift | More realistic | Smears the voxel map; adds a confound to the comparison |
| vSLAM (as in the proposal) | Most faithful to the proposal | Weeks of work on its own; smoke would degrade it too, mixing two effects |

**Decision:** ground-truth base and camera pose. The map and the phone
location stay unknown. The report must state this as a simplification and
explain the change from the proposal.

---

## Open decisions ⏳

- **OPEN-1: Base planner when the map is unknown** (voxel map built from
  depth). Options: A* / D* Lite on a 2D projection of the voxel map (treating
  unknown space as free or as occupied); RRT-Connect against the voxel map,
  with replanning as the map grows. Jhanny expects A*; decide when the voxel
  map exists.
- **OPEN-2: Terminal base placement for reachability.** Options: fixed
  offset from the phone; sample base poses and keep those where IK finds a
  collision-free grasp; a reachability map precomputed offline.
- **OPEN-3: Viewpoint selection** (information gain vs. travel cost), and how
  the audio bearing/loudness enters it (prior over target location, Bayes
  filter, ...).
- **OPEN-4: Arm motion planning:** collision-checked trajectory optimization
  (proposal) vs. RRT in joint space vs. KinematicTrajectoryOptimization.
- **OPEN-5: Grasp selection:** antipodal sampling (proposal) and how
  candidates are scored.
