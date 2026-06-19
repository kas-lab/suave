#!/bin/bash
set -e

CURDIR=$(pwd)

cd ../../

colcon build --symlink-install --packages-select \
    suave \
    suave_bt \
    suave_metacontrol \
    suave_metrics \
    suave_missions \
    suave_monitor \
    suave_msgs \
    suave_none \
    suave_random \
    suave_runner \
    suave_tools

source install/setup.bash

colcon test --event-handlers console_direct+ --packages-select \
    suave \
    suave_bt \
    suave_metacontrol \
    suave_metrics \
    suave_missions \
    suave_monitor \
    suave_msgs \
    suave_none \
    suave_random \
    suave_runner \
    suave_tools

cd "$CURDIR"