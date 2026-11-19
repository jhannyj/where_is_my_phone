# Where Is My Phone?

Audio-guided search and grasping with a mobile manipulator (KUKA iiwa 14 on a
skid-steer mobile base, WSG 50 gripper, wrist RGB-D camera) under visual
occlusion, simulated in Drake. 6.4210 final project, Team 2.1.

**Current status:** the robot can drive to the phone, grasp it and lift it,
with the phone's pose known (16/16 random poses on the coffee table).
Perception (voxel map, phone detection, smoke, audio) is next. Every
robotics design choice and its reasoning is in
[`docs/DECISIONS.md`](docs/DECISIONS.md).

---

## 1. Pick your setup path

| Your machine | Use |
|---|---|
| Mac with **Apple Silicon** (M1–M4) on **macOS 26** or newer | [Native setup](#2a-native-setup-apple-silicon-mac-linux-wsl) |
| Linux, x86_64 or ARM, **Ubuntu 22.04+** (glibc 2.34+) | [Native setup](#2a-native-setup-apple-silicon-mac-linux-wsl) |
| Windows | [Native setup](#2a-native-setup-apple-silicon-mac-linux-wsl) inside [WSL 2](https://learn.microsoft.com/en-us/windows/wsl/install) with Ubuntu 22.04+ |
| **Intel Mac**, or Apple Silicon on macOS older than 26, or anything else | [Docker setup](#2b-docker-setup-any-machine) |

Why: we pin `drake==1.58.0`, so everyone runs the same API, and that version
only ships wheels for Apple Silicon macOS 26+ and Linux (glibc 2.34+), with
Python 3.12+. Not sure which Mac you have? Apple menu → About This Mac
("Chip: Apple M…" means Apple Silicon).

---

## 2a. Native setup (Apple Silicon Mac, Linux, WSL)

Requires `git` and [uv](https://docs.astral.sh/uv/getting-started/installation/)
(`curl -LsSf https://astral.sh/uv/install.sh | sh`, then open a new terminal).

```sh
git clone https://github.com/jhannyj/where_is_my_phone.git
cd where_is_my_phone
uv venv --python 3.13
source .venv/bin/activate
uv pip install -r requirements.txt
```

Check the install (should print `drake 1.58.0` and then `5 passed`):

```sh
python -c "import pydrake.all, manipulation; from importlib.metadata import version; print('drake', version('drake'))"
python -m pytest -q tests
```

Every later session: `cd where_is_my_phone && source .venv/bin/activate`.
Run all commands **from the repo root** (the code finds models relative to it).

> **Path note:** Drake downloads robot models (iiwa, gripper) with a script
> inside the venv, and fails if the venv path contains spaces or special
> characters. If your clone lives under such a path, put the venv elsewhere:
> `uv venv --python 3.13 ~/venv_wimp && source ~/venv_wimp/bin/activate`.

## 2b. Docker setup (any machine)

1. Install [Docker Desktop](https://docs.docker.com/desktop/) (on Mac, pick
   the build for your chip) and **start it**: the whale icon in the menu bar
   must say it's running.
2. Clone the repo:
   ```sh
   git clone https://github.com/jhannyj/where_is_my_phone.git
   cd where_is_my_phone
   ```
3. Prefix every command in this README with `./docker_run.sh`:
   ```sh
   ./docker_run.sh python -m pytest -q tests
   ```
   The first run builds the image, which downloads about 1 GB and takes a few
   minutes. The first simulation also downloads the robot models into a
   cached Docker volume. Later runs start in seconds.

Notes:
- The repo is mounted into the container, so code edits on your machine take
  effect immediately. Only rebuild after changing `requirements.txt` or the
  `Dockerfile`: `REBUILD=1 ./docker_run.sh python -m pytest -q tests`.
- Meshcat (the 3D viewer) is published on **http://localhost:7000**.
- Inside the container the repo is at `/work`, so "replay saved to
  /work/outputs/…" means the `outputs/` folder of your clone.
- `./docker_run.sh bash` gives you a shell inside the container.
- **`docker: command not found` on Mac** even though Docker Desktop is
  running: add its CLI to your PATH:
  `export PATH="$PATH:/Applications/Docker.app/Contents/Resources/bin"`
  (put it in `~/.zshrc` to keep it).
- On Windows, run `docker_run.sh` from a WSL terminal, or use the native
  setup inside WSL instead.

---

## 3. Running things

Native: run the commands as shown. Docker: put `./docker_run.sh` in front.

### Fetch the phone: drive, grasp, lift (the main demo)

```sh
python -m wimp.fetch_phone                        # phone at its default pose
python -m wimp.fetch_phone --phone 2.7 0.2 135    # phone at x, y (m), yaw (deg) on the coffee table
python -m wimp.fetch_phone --verbose              # also print a grasp timeline
python -m wimp.fetch_phone --wait                 # keep the live viewer open at the end
```

It plans everything first (usually 2–10 s; occasionally about 40 s when
the parking search has to try many spots), then simulates the roughly 45 s
motion (about 10–30 s of wall time), then prints a verdict:

```
phone z 0.459 -> 0.605 (lift 14.6 cm) | phone-to-pads 0.7 cm | ...
SUCCESS (lifted >= 10 cm: True, in hand: True)
replay saved to .../outputs/fetch_phone.html
```

Valid `--phone` positions lie on the coffee table: x in [2.55, 3.05],
y in [-0.55, 0.55] (the table top spans x 2.5–3.1, y −0.6–0.6, z = 0.45).
`--seed N` changes the random sampling in the planners.

### Drive to a goal (base navigation only)

```sh
python -m wimp.drive_to_goal                          # default goal
python -m wimp.drive_to_goal --goal 1.1 3.9 1.57      # x, y (m), yaw (rad)
```

It prints the planning stats, the final goal error, the maximum sideways
speed (should be about 0: the base is skid-steer), and whether the executed
motion kept a 3 cm clearance. The planned path is drawn on the floor:
orange is the raw RRT path, green the shortcut one.

### View the robot model alone

```sh
python view_robot.py
```

Opens the robot in an empty world with joint sliders (Meshcat, **Open
Controls**). Stop it with Ctrl+C.

### Tests

```sh
python -m pytest -q tests
```

Unit tests for RRT-Connect and the skid-steer steering. These don't need a
simulation and run in about a second.

---

## 4. Watching the results

Every demo saves a **replay** you can open in any browser, with no server
needed:

- `outputs/fetch_phone.html`
- `outputs/drive_to_goal.html`

Double-click the file, or `open outputs/fetch_phone.html` on Mac.
`outputs/` is not committed; each person generates their own.

In the viewer:
- **Rotate** by dragging, **zoom** by scrolling, **pan** by right-dragging.
- **Open Controls** (top right) → **Animations**: play, pause, and a time
  slider. For the fetch demo, the grasp happens near the end (roughly the
  last 10 s).
- Under **Scene** in the same panel you can hide objects, such as the walls.

**Live viewer:** while a script runs, Meshcat is also live at the URL it
prints (usually http://localhost:7000; natively Drake may pick 7001, 7002…
if 7000 is busy). Add `--wait` so it stays open after the run, then press
Enter in the terminal to quit.

---

## 5. Troubleshooting

| Problem | Fix |
|---|---|
| `uv pip install` says no `drake` wheel matches your platform | Your machine isn't supported natively (see section 1). Use Docker. |
| `PackageMap: Downloading ...` then an error | The first run needs internet to download `drake_models`. Natively, also check the venv path note in 2a. |
| `ModuleNotFoundError: No module named 'wimp'` | Run from the repo root, with `python -m wimp.…` (not `python wimp/…`). |
| Docker: `Cannot connect to the Docker daemon` | Start Docker Desktop and wait until it says it's running. |
| Docker: port 7000 already in use | Stop the other Meshcat or container (`docker ps`, then `docker stop <id>`). |
| `FAILURE` or `no reachable base pose found` | Try another `--seed`, and check that `--phone` is on the table (section 3). Then report it, with the command, to whoever owns that part. |

---

## 6. Repo map

| Path | What |
|---|---|
| `living_room_scene/` | Room, furniture and phone models + `living_room_scene.dmd.yaml` (package `living_room`). The room spans x, y in [−5, 5] m. |
| `models/` | Robot: `mobile_base.sdf`, `mobile_manipulator.dmd.yaml` (base + iiwa 14 + WSG + ZED camera), `zed_camera.sdf`; `nav_scene.dmd.yaml` = room + robot (package `custom_models`). |
| `wimp/scene.py` | Loads the scene; robot start pose, arm stow posture. |
| `wimp/planning/` | `rrt_connect.py` (generic RRT-Connect + shortcutting), `diff_drive.py` (skid-steer turn-drive-turn steering), `base_planner.py`, `arm_planner.py`, `planning_scene.py` (shared collision checker). |
| `wimp/grasping/` | `antipodal.py` (grasp sampling), `gripper_checker.py`, `kinematics.py` (IK, differential IK), `base_placement.py` (where to park). |
| `wimp/sim.py` | Physics simulation with joint PD control. |
| `wimp/fetch_phone.py`, `wimp/drive_to_goal.py` | The demos above. |
| `view_robot.py` | Robot model viewer. |
| `tests/` | Unit tests. |
| `docs/DECISIONS.md` | Every robotics choice: options, decision, reasoning, planned experiments. |
| `CLAUDE.md` | Project goal and working rules (also read by Claude Code). |
| `Dockerfile`, `docker_run.sh` | Docker setup. |
