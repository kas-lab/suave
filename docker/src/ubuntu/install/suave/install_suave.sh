#!/usr/bin/env bash
set -ex

source /opt/ros/humble/setup.bash
cd ~/pipeline_ws/
colcon build --symlink-install --executor sequential --parallel-workers 1
echo 'source ~/pipeline_ws/install/setup.bash' >> ~/.bashrc


sudo apt-get install -y python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
pip3 install PyYAML mavproxy --user
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

cd ~/pipeline_ws/src/mavros/mavros/scripts

sudo ./install_geographiclib_datasets.sh
