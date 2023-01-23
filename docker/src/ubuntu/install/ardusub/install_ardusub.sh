#!/usr/bin/env bash
set -ex
cd $HOME
git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot

git checkout 9f1c4df

git submodule update --init --recursive

#install-prereqs-ubuntu.sh from ardupilot

sudo apt install python-is-python3

sudo pip3 install future

sudo pip3 install -U mavproxy pymavlink
sudo rm ~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
sudo wget -P ~/ardupilot/Tools/environment_install/ https://raw.githubusercontent.com/ArduPilot/ardupilot/c623ae8b82db4d7e195f4b757e2ae5d049f941e5/Tools/environment_install/install-prereqs-ubuntu.sh
export USER=kasm-user
sudo chmod +x ~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y

cd ~/ardupilot
./waf configure
./waf clean