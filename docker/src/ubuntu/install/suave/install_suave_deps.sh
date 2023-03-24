#!/usr/bin/env bash
set -ex

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
