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
