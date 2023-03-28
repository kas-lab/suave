#!/bin/bash

if [ $# -lt 1 ]; then
	echo "usage: $0 gui"
	echo example:
	echo "  "$0 "[true | false]"
	exit 1
fi

GUI=""

if [ "$1" == "true" ];
then
    GUI='--console'
else
  if [ "$1" == "false" ];
  then
      GUI=''
  else
    echo "gui argument invalid"
    exit 1
  fi
fi

cd /tmp
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON $GUI
