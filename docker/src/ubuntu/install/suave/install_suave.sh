#!/usr/bin/env bash
set -ex

source /opt/ros/humble/setup.bash
cd ~/pipeline_ws/
colcon build --symlink-install --executor sequential --parallel-workers 1
echo 'source ~/pipeline_ws/install/setup.bash' >> ~/.bashrc
