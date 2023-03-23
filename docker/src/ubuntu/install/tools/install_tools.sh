#!/usr/bin/env bash
set -e

echo "Install some common tools for further installation"
apt-get update
# Update tzdata noninteractive (otherwise our script is hung on user input later).
DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get -y install tzdata
apt-get install -y vim wget locales wmctrl software-properties-common mesa-utils
apt-get clean -y

echo "generate locales für en_US.UTF-8"
locale-gen en_US.UTF-8

add-apt-repository ppa:kisak/kisak-mesa
apt full-upgrade -y
