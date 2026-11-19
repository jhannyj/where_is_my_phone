# Linux container for running Drake on machines without native wheels
# (e.g. Intel Macs: Drake only ships macOS wheels for Apple Silicon).
# Usage: see README.md ("Running with Docker").
FROM python:3.13-slim-bookworm

# Runtime libraries Drake's wheel expects on Debian/Ubuntu (rendering, X libs).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libx11-6 libsm6 libxt6 libglib2.0-0 libgl1 libegl1 libxrender1 \
        libgomp1 git curl ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && ln -sf /usr/local/bin/python3 /usr/bin/python3
# (Drake's package downloader for drake_models runs /usr/bin/python3.)

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt pytest

# Meshcat binds to all interfaces inside the container so the host browser
# can reach it through the published port (see wimp/scene.py: start_meshcat).
ENV MESHCAT_HOST="*" \
    MESHCAT_PORT=7000 \
    PYTHONUNBUFFERED=1

WORKDIR /work
EXPOSE 7000
