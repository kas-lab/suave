#!/usr/bin/env bash
set -ex

source /opt/ros/humble/setup.bash
cd ~/suave_ws/
colcon build --symlink-install
echo 'source ~/suave_ws/install/setup.bash' >> ~/.bashrc
