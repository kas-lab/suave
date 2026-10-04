#!/bin/bash
set -e
CURDIR=$(pwd)

# Load pinned git commit SHAs and forward them as build-args.
# Python package versions are in requirements.txt (copied into the images).
set -a; source docker/versions.env; set +a

VERSION_ARGS=(
    --build-arg ARDUSUB_COMMIT
    --build-arg ARDUPILOT_GAZEBO_COMMIT
)

docker build -t kasm-jammy:latest -f docker/dockerfile-kasm-core-jammy .
docker build -t suave:latest --build-arg BASE_IMAGE=kasm-jammy:latest "${VERSION_ARGS[@]}" -f docker/dockerfile-suave .
docker build -t suave-headless:latest "${VERSION_ARGS[@]}" -f docker/dockerfile-suave-headless .

cd "$CURDIR"
