#!/usr/bin/env bash
set -ex

mkdir -p ~/suave_ws/src
cd ~/suave_ws

wget https://raw.githubusercontent.com/kas-lab/suave/main/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/bluerov2_ignition/models:$HOME/suave_ws/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/remaro_worlds/models:$HOME/suave_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

source /opt/ros/humble/setup.bash
cd ~/suave_ws/

sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install -y python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

cd ~/suave_ws/src/mavros/mavros/scripts

sudo ./install_geographiclib_datasets.sh
