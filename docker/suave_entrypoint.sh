#!/bin/bash
# Entrypoint of the headless image: sources the SUAVE workspace (and with it
# ROS 2) so commands work without an interactive shell, e.g.
# `docker run <image> python3 ...`. `docker exec` does not use it.
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$HOME/suave_ws/install/setup.bash" --
exec "$@"
