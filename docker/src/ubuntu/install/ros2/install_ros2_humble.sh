#!/usr/bin/env bash
set -ex

apt update && apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y

apt install -y ros-humble-ros-base

apt install -y python3-colcon-common-extensions

apt install -y python3-rosdep

apt install -y python3-vcstool
