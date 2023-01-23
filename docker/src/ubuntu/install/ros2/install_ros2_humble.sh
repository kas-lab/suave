#!/usr/bin/env bash
set -ex

apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update

#sudo apt install -y ros-humble-desktop=0.10.0-1jammy.20220922.190042
sudo apt install -y ros-humble-desktop

sudo apt install -y python3-colcon-common-extensions=0.3.0-1

sudo apt install -y python3-rosdep=0.22.1-1

sudo apt install -y python3-vcstool=0.3.0-1
