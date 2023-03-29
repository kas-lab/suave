#!/bin/bash
source ~/.bashrc
source ~/suave_ws/install/setup.bash
if [ $# -lt 1 ]; then
	echo "usage: $0 gui"
	echo example:
	echo "  "$0 "[true | false]"
	exit 1
fi

GUI=""

if [ "$1" == "true" ] || [ "$1" == "false" ];
then
    GUI=$1
else
    echo "gui argument invalid"
    exit 1
fi

ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0 gui:=$GUI
