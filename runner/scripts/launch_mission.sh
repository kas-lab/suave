#!/bin/bash
source ~/.bashrc
source ~/suave_ws/install/setup.bash

if [ $# -lt 1 ]; then
	echo "usage: $0 adaptation_manager mission_type"
	echo example:
	echo "  "$0 "[metacontrol | random | none] [time | distance]"
	exit 1
fi

MANAGER=""
MTYPE=""

if [ "$1" == "metacontrol" ] || [ "$1" == "random" ] || [[ "$1" == "none" ]];
then
    MANAGER=$1
else
    echo "adaptation_manager invalid or missing"
    exit 1
fi

if [ "$2" == "time" ];
then
    MTYPE="time_constrained_mission"
else
    if [ "$2" == "distance" ];
    then
        MTYPE="const_dist_mission"
    else
        echo "mission_type invalid or missing"
        exit 1
    fi
fi

ros2 launch suave_missions mission.launch.py adaptation_manager:=$MANAGER mission_type:=$MTYPE result_filename:=$3
