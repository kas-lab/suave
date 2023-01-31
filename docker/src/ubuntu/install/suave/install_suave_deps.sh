#!/usr/bin/env bash
set -ex

mkdir -p ~/pipeline_ws/src
cd ~/pipeline_ws

wget https://raw.githubusercontent.com/kas-lab/suave/main/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/pipeline_ws/src/bluerov2_ignition/models:$HOME/pipeline_ws/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/pipeline_ws/src/remaro_worlds/models:$HOME/pipeline_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

source /opt/ros/humble/setup.bash
cd ~/pipeline_ws/

sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y
