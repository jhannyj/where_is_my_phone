# Where Is My Phone?

Audio-guided search and grasping with a mobile manipulator (KUKA iiwa 14 on a
mobile base) under visual occlusion, simulated in Drake.
6.4210 final project, Team 2.1.

## Setup

Requires `git` and [uv](https://docs.astral.sh/uv/). Drake has no native Windows
build, so on Windows use [WSL](https://learn.microsoft.com/en-us/windows/wsl/install).

```sh
git clone <repo-url> where_is_my_phone
cd where_is_my_phone
uv venv --python 3.13
source .venv/bin/activate
uv pip install -r requirements.txt
```

Check the install:

```sh
python -c "import pydrake, manipulation; print('ok')"
```

**Path note:** Drake downloads robot models using a script inside the venv and
fails if the venv path contains spaces or special characters. If your clone
is under such a path, create the venv elsewhere, e.g.
`uv venv --python 3.13 ~/venv_where_is_my_phone`.

## Running with Docker (Intel Macs, or anywhere without Drake wheels)

Drake only ships macOS wheels for Apple Silicon. On an Intel Mac, install
[Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/)
(the "Mac with Intel chip" build) and run commands in the container:

```sh
./docker_run.sh python -m wimp.drive_to_goal --wait   # Meshcat at http://localhost:7000
./docker_run.sh python -m pytest tests
```

The first run builds the image (a few minutes). Set `REBUILD=1` after
changing `requirements.txt`.

## Drive to a goal (known map)

```sh
python -m wimp.drive_to_goal                    # default goal next to the phone
python -m wimp.drive_to_goal --goal 3.0 3.5 1.57 --seed 3
```

Plans an (x, y, yaw) path with RRT-Connect, simulates the mobile
manipulator tracking it, and saves a Meshcat replay to `outputs/drive_to_goal.html`.
Design choices are documented in `docs/DECISIONS.md`.

## Fetch the phone (known pose)

```sh
python -m wimp.fetch_phone                       # phone at its scene pose
python -m wimp.fetch_phone --phone 2.7 0.2 135   # x, y, yaw (deg) on the coffee table
```

Samples antipodal grasps, picks a parking pose by expected reachability,
drives there (RRT-Connect), moves the arm to pre-grasp (joint-space
RRT-Connect), approaches and lifts with differential IK, and simulates it all.
It reports SUCCESS if the phone ends up at least 10 cm higher and still
between the fingers. The replay is saved to `outputs/fetch_phone.html`; add
`--verbose` for a grasp timeline.
