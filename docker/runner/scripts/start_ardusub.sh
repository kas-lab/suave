#!/bin/bash
CURDIR=`pwd`
cd ~/Desktop/
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON
cd $CURDIR
