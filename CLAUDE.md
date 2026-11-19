# Where Is My Phone? — 6.4210 Final Project (Team 2.1)

Audio-guided search and grasping with a mobile manipulator (KUKA iiwa on a
mobile base) under visual occlusion ("smoke"), simulated in Drake. Full
proposal: `6.4210_Group 2.1_Proposal.pdf`.

## The goal

Compare two methods for finding and grasping a phone-shaped cuboid hidden in a
living room:

- **(A) vision-only search**: viewpoint selection from a voxel map
  (free / occupied / unknown), color-segmentation detection.
- **(B) audio-visual search**: same as A plus a noisy bearing + loudness
  estimate toward the phone (Gaussian noise + outliers).

Each method runs at smoke levels 0.0 / 0.3 / 0.6 / 0.9, ~50 trials each.
Metrics: success rate (find, navigate, lift 10 cm without collision),
time-to-grasp, viewpoints visited, fraction of room explored before detection.

Pipeline per the proposal: perception (depth → voxel map, color segmentation,
smoke noise, mic at the end-effector) → search & motion planning (viewpoint
selection, base navigation, arm motion) → grasping (antipodal candidates,
optimization IK, diff-IK approach) → state machine coordinating everything.

## How we work (IMPORTANT — read before making changes)

1. **Every robotics-algorithm choice must be explainable by the team.** Course
   policy: AI coding help is allowed, but we must own and justify every
   robot-level design choice (e.g. RRT vs trajectory optimization, antipodal vs
   force-closure grasp scoring). Implementation details (data structures,
   Drake API calls) do not need this treatment.
2. **When such a choice comes up, stop and present the options** to the user
   with trade-offs and a recommendation, and let them decide. Do not silently
   pick an algorithm.
3. **Record every decision in [`docs/DECISIONS.md`](docs/DECISIONS.md)**:
   the options considered, what was chosen, why, and what would make us
   revisit it. This file is what we will draw on for the report and for
   answering staff questions, so keep it current.
4. Code written as a stand-in (e.g. the placeholder robot) must be clearly
   marked as such so it can be swapped for the teammates' real components.

## Current target

Aim directly for the **11/20 milestone**: find and grasp the phone using
vision + audio (method B). The 10/30 "known map" milestone is not a
deliverable, but its base planner is a building block and already exists.
The vision-only search (11/13 milestone, method A) is NOT optional: it is
the baseline the course requires us to compare against, and it shares almost
all of its code with method B. Simplifications so far: the robot knows its
own pose exactly (no vSLAM; see D-010).

## Team and ownership (from the proposal)

- **Akua Yeboah**: Drake environment, obstacles, image processing, point
  clouds, state machine integration, target detection.
- **Chenise Harper**: robot description (the real mobile manipulator), sound
  model, smoke simulation, grasp generation, acoustic guidance.
- **Jhanny Jimenez Feliz**: voxel mapping, base and arm motion, IK, viewpoint
  selection, experiment logging.

## Repo layout

- `living_room_scene/` — room + furniture SDFs and `living_room_scene.dmd.yaml`
  (package name `living_room`; x, y in [-5, 5] m, floor at z = 0).
- `models/` — package `custom_models`: the robot (`mobile_base.sdf`,
  `mobile_manipulator.dmd.yaml` = base + iiwa 14 + WSG gripper + ZED camera,
  `zed_camera.sdf`) and `nav_scene.dmd.yaml` (living room + robot).
  `view_robot.py` (repo root) shows the robot alone.
- `wimp/` — Python code. `planning/rrt_connect.py` (generic RRT-Connect +
  shortcutting), `planning/base_planner.py` ((x, y, yaw) planning with Drake
  collision checking, time parameterization), `drive_to_goal.py` (milestone 1
  demo).
- `tests/` — pytest tests.
- `docs/DECISIONS.md` — algorithm decision log (see above).
- `README.md` — environment setup (uv + Python 3.13 venv, `drake`,
  `manipulation`).
- Jhanny's machine is an **Intel Mac: Drake does not run natively there**.
  Run everything through `./docker_run.sh <cmd>` (Meshcat on localhost:7000).
  The docker CLI lives in `/Applications/Docker.app/Contents/Resources/bin`
  (add it to PATH if `docker` is not found).
