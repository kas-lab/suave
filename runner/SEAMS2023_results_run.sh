#!/bin/bash

MTYPE="time"
GUI="false"

MANAGER="none"
./runner.sh $GUI $MANAGER $MTYPE 20

MANAGER="metacontrol"
./runner.sh $GUI $MANAGER $MTYPE 20

MANAGER="random"
./runner.sh $GUI $MANAGER $MTYPE 20