#!/bin/bash

# Configuration.
apt update -qq
apt install -qq -y lsb-release wget curl build-essential

# Garden only has nightlies for now
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

GZ_DEPS="libgz-sim7-dev"

# Dependencies.
# echo "deb http://packages.ros.org/ros2-testing/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-testing.list
# curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update -qq
apt-get install -y $GZ_DEPS
