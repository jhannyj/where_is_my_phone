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

*Updated by D-015:* the distance is now the cost of the turn-drive-turn
motion between two poses: drive length + w × (|turn 1| + |turn 2|).

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

*2026-11-18 addendum (grasping):*
- **Gripper force:** the finger PD setpoint is commanded fully closed, past
  the phone. The PD error then acts as a squeeze force (≈ 800 N/m × 3.4 cm
  ≈ 27 N per finger, under the 80 N effort limit), so position control
  doubles as simple force control.
- **Contact solver:** `similar`, a SAP-family solver, so the PD actuators
  still work. Plain `sap` let the light phone settle about 8 mm into the
  table even with an explicit point-contact stiffness. `similar` and `lagged`
  rest it correctly.
- **Phone contact:** the phone's contact parameters (μ = 1.2, stiffness
  1e5 N/m) were written as Gazebo `<ode>` tags, which Drake ignores. They are
  now `drake:proximity_properties`.

---

## D-008 Goal base pose (superseded by D-014)

For the first drive-to-goal demo the goal pose was hand-picked.
`fetch_phone` now chooses it with D-014.

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

## D-011 Phone placement and size for grasping ✅

*2026-11-18 · Decided by: Jhanny*

**Context:** the WSG 50 opens to 10.7 cm between its pads, so on a phone
lying flat it can only pinch across the 7.5 cm width, top-down. With an
0.8 cm phone the fingertips must stop within about 3 mm of the support
surface.

| Option | Pros | Cons |
|---|---|---|
| Floor, 1.5 cm thick | ~1 cm fingertip margin; "dropped phone" story | Lower for the arm to reach |
| Floor, 0.8 cm (original) | Most realistic | Fingertips must stop within ~3 mm of the floor; fragile grasps would muddy the vision-vs-audio comparison |
| **Coffee table, 1.5 cm thick** | Comfortable reach for the arm; ~1 cm fingertip margin | Less of a "lost phone" story |

**Decision:** 7.5 × 15 × 1.5 cm ("phone in a rugged case") on the coffee
table, z = 0.4585 m. `wimp.fetch_phone --phone X Y YAW` places it anywhere on
the table. Moving it to the floor later is a one-line change.

---

## D-012 Grasp selection ✅

*2026-11-18 · Method decided by Jhanny; scoring details proposed by Claude 🟡*

| Option | Pros | Cons |
|---|---|---|
| **Antipodal sampling** (course ch. 5) | Works on a point cloud, so it carries over to perception; matches the proposal; every candidate has a friction-cone justification | Must sample many candidates; needs a collision filter |
| Hand-designed top-down grasp from the known box pose | Simplest; very reliable | Needs the exact object pose and shape; doesn't generalize to a noisy cloud |

**Decision:** antipodal sampling (`wimp/grasping/antipodal.py`):
1. Surface points and normals: from the phone model now (3000 points), from
   the segmented RGB-D cloud later.
2. For a point with normal n, the closing axis is a = −n. The opposite
   contact is the farthest cloud point within 4 mm of that line.
3. Keep the pair if n_opposite · a ≥ cos 15°. That is a stricter cone than
   μ = 1.2 allows (50°), which leaves margin for noisy normals later.
4. Width must be at most 9 cm (pads open to 10.7 cm).
5. Approach directions: 8 angles about the closing axis. Contact line placed
   3, 6 or 9 mm toward the fingertips from the pad centre, so the tips stick
   out less below the phone.
6. Score = antipodal quality + 1.0 × (approach · down) − 5 × (distance of the
   grasp centre from the centre of mass, in m). The top-down term reflects
   that the phone lies on a support surface. The centre-of-mass term reduces
   the torque that makes the phone slip during the lift.
7. **Floating-gripper collision filter** (`gripper_checker.py`): the gripper
   alone, fingers open, at the grasp and pre-grasp poses, must clear the
   environment by 2 mm. Without this, IK wasted its budget on grasps whose
   fingertips were inside the table (about 98% of candidates are rejected).

---

## D-013 Arm motion: IK + joint-space RRT-Connect + differential IK ✅

*2026-11-18 · Decided by: Jhanny*

| Option | Pros | Cons |
|---|---|---|
| **Optimization IK for pre-grasp/grasp + RRT-Connect in 7-D joint space (stow → pre-grasp) + diff IK for approach and lift** | Each piece is a course method; reuses our RRT-Connect; approach and lift are straight Cartesian lines | Joint-space paths are not smooth (shortcut + stop-and-go timing) |
| KinematicTrajectoryOptimization stow → pre-grasp | Smooth, optimized | Slower; can fail to converge; harder to debug |
| IK keyframes without collision checks in between | Fastest to build | Not collision-safe in general |

**Details** (proposed by Claude 🟡):
- **IK** (`kinematics.py`): Drake `InverseKinematics` with the base fixed.
  Position tolerance 0.5 mm, orientation 0.005 rad, joint limits; cost
  ‖q − q_seed‖². Collision is *not* an IK constraint. Each solution is checked
  with the collision checker and rejected if it collides. Reason: Drake's
  minimum-distance constraint also counts the phone resting on the table as
  a violation. *Lesson learned:* a 2 mm / 0.02 rad tolerance let the
  seed-cost pull the fingertips about 4 mm lower, into the table.
- **Pre-grasp:** 10 cm back along the approach axis.
- **Diff IK:** damped least squares qdot = Jᵀ(JJᵀ + λ²I)⁻¹V, with λ = 0.01,
  V = feed-forward + 5 × pose error, and |qdot| ≤ 1.5 rad/s. Integrated
  offline at 10 ms, then tracked by the joint PD. Approach takes 2 s; lift is
  15 cm in 2.5 s.

---

## D-014 Base placement: expected reachability over the target belief ✅

*2026-11-18 · Recommended by Claude after Jhanny asked for a placement method
that blurry vision would hurt and audio would help. Accepted by Jhanny "for
now", to revisit with experiments.*

| Option | Pros | Cons |
|---|---|---|
| Fixed offset in front of the target | Trivial | Fails near furniture or at awkward phone orientations; ignores uncertainty |
| Sample bases + IK check on the point estimate | Principled for a known pose | Ignores uncertainty: a spot that reaches the estimate may miss the true phone |
| **Sample bases, score by expected reachability over belief particles** | Equals the previous option when the pose is known; with a spread-out belief it picks the spot covering the most probable phone locations | IK cost grows with the number of particles |
| Offline reachability map | Fast at runtime | Must be precomputed for the real robot; still needs the belief |

**Method** (`base_placement.py`): candidate bases on rings 0.6–0.9 m from
the belief mean, 24 angles, and 3 headings (facing the phone or ±90°). Keep
the collision-free ones and sort by travel distance. Score = fraction of
belief particles with a feasible grasp (IK pre-grasp and grasp,
collision-free, plus a collision-free diff-IK approach). Stop at the first
base with score 1 (within the tie tolerance), or after 40 bases. A base is only accepted if the base
planner can actually drive there. In 1 of 15 test poses the best spot was a
collision-free pocket under the table edge, between the couch and a table
leg, that the base could not get into.

**Why it fits the project:** with smoke the belief is spread out, so
reachability drops and the robot must re-observe and re-park. Audio
sharpens the belief, so one parking spot suffices more often. "Re-parks"
and "reachability at first park" become metrics that link perception
quality to manipulation success.

**Refinements, decided 2026-11-18 (Jhanny, "go with the recommendations
for now"):**
- **Ranking:** reach first; reach scores within 0.1 of each other count as
  ties, and the base with the shorter drive wins.
- **Low reach:** if the best reach is below 0.6, still park there,
  re-observe from there, update the belief and re-plan. Each re-plan counts
  as a *re-park* in the logs.
- **Speed:** a reach-band prefilter skips grasps whose gripper frame is
  outside 0.15–1.10 m of the shoulder (iiwa joint 2), with no IK solve.
  This is a necessary condition, padded so it never rejects a reachable
  grasp. Only the top 3 grasps per base go to IK.
- **Ablation:** run audio-for-search-only vs. audio-for-search-and-parking
  (E-1).

---

## D-015 Base motion model: skid-steer (turn in place + straight, reversing allowed) ✅

*2026-11-18 · Decided by: Jhanny (after noticing the base slid diagonally)*

| Option | Pros | Cons |
|---|---|---|
| **Skid-steer / differential drive: turn in place, drive straight, forwards or backwards** | Matches a four-wheeled indoor base (e.g. a Clearpath Husky); RRT-Connect stays, only the edges change; any sequence of these motions is executable | Stops to turn, so slower than smooth curves |
| Car-like (Ackermann, minimum turning radius) | Realistic for cars | Can't turn in place; needs Dubins/Reeds-Shepp edges and multi-point turns near furniture; much more work |
| Holonomic (mecanum/omni wheels) | No change | Doesn't match the four-wheeled model |

**Reversing allowed** ✅: the base can back out of a parking spot beside the
table instead of turning around in a tight space.

**How:** `wimp/planning/diff_drive.py`. The steering function between poses
a and b is rotate-translate-rotate: turn to the driving heading h, drive
straight, turn to b's yaw. h is the segment direction or that + π,
whichever needs less total turning. RRT-Connect's EXTEND/CONNECT, the edge
collision checks, and shortcutting all use this steering, so every checked
edge is a feasible skid-steer motion. Before timing, the path is expanded
into pure-turn and pure-drive primitives, each timed with the D-006 cubic
profile.

**Check:** in simulation the measured sideways speed stays at most 2 mm/s
(PD tracking error) against 0.5 m/s forward speed. Batch of 16 random
phone poses on the table: 16/16 grasped and lifted 14.2–14.9 cm.

**Note:** in the simulator the base still moves through virtual x/y/yaw
joints, so the wheels are not simulated. We plan and command only
skid-steer-feasible motions; that is the kinematic model of the base.

---

## Planned experiments (to revisit once real challenges show up)

These choices were made to make progress. Jhanny noted that, since this is
an experiment, we should test alternatives when we hit real problems:

- **E-1 Audio ablation (D-014):** audio used for search only vs. for search
  *and* base placement. Separates the two ways audio could help.
- **E-2 Base placement variants:** expected reachability vs. point-estimate
  sample+IK vs. fixed offset, under smoke.
- **E-3 Tie tolerance / commit threshold** (0.1 / 0.6, D-014): sensitivity
  check if re-parks turn out to be frequent.
- **E-4 Grasp scoring** (D-012): does the centre-of-mass term or the
  top-down term matter once grasps come from a noisy cloud?
- **E-6 In-hand twist:** the WSG pads touch through 1 mm contact spheres,
  so the phone can pivot about 10° about the vertical as the fingers close
  (the joint gap reaches about 9 cm instead of about 7.4 cm). Every lift
  still succeeds (16/16), but with noisy grasp poses this could cause drops.
  Options if it does: hydroelastic contact or wider pad geometry.
- **E-7 Motion quality:** after watching the replay, Jhanny noted that the
  motion "doesn't look too optimal". Likely causes: RRT-Connect paths are
  feasible but not shortest (random shortcutting only partly fixes this);
  stop-and-go timing (D-006) stops at every corner; turn-drive-turn edges
  (D-015) add turns. Options: RRT* or an A*-based base planner; smoother
  timing (e.g. TOPP-RA) that turns while driving where it is safe; a smarter
  parking spot that shortens the drive. Total drive time feeds the
  time-to-grasp metric, so this matters for the comparison.
- **E-5 Base planner on the unknown map** (OPEN-1): A* on the 2D projected
  voxel map vs. RRT-Connect against the voxel map.

---

## Open decisions ⏳

- **OPEN-1: Base planner when the map is unknown** (voxel map built from
  depth). Options: A* / D* Lite on a 2D projection of the voxel map (treating
  unknown space as free or as occupied); RRT-Connect against the voxel map,
  with replanning as the map grows. Jhanny expects A*; decide when the voxel
  map exists.
- **OPEN-3: Viewpoint selection** (information gain vs. travel cost), and how
  the audio bearing/loudness enters it (prior over target location, Bayes
  filter, ...).
- **OPEN-6: Belief representation for the phone pose** (particles vs.
  Gaussian vs. occupancy grid over candidate locations). D-014 already
  accepts particles.
