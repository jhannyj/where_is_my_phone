#!/usr/bin/env bash
# Run a command inside the Drake container with this repo mounted at /work.
#   ./docker_run.sh python -m wimp.drive_to_goal
#   ./docker_run.sh bash            # interactive shell
# Meshcat is published on http://localhost:7000 .
set -euo pipefail
cd "$(dirname "$0")"
IMAGE=wimp-drake

if ! docker image inspect "$IMAGE" >/dev/null 2>&1 || [ "${REBUILD:-0}" = 1 ]; then
  docker build -t "$IMAGE" .
fi

TTY_FLAGS="-i"
[ -t 0 ] && TTY_FLAGS="-it"

# drake_models (iiwa etc.) is downloaded on first use; cache it in a volume.
docker run --rm $TTY_FLAGS \
  -p 7000:7000 \
  -v "$PWD":/work \
  -v wimp-drake-cache:/root/.cache \
  "$IMAGE" "$@"
