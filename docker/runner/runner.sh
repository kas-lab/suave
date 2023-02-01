#!/bin/bash

if [ $# -lt 1 ]; then
	echo "usage: $0 gui adaptation_manager mission_type"
	echo example:
	echo "  "$0 "[true | false]"
	exit 1
fi

GUI=""
MANAGER=""
MTYPE=""

if [ "$1" == "true" ] || [ "$1" == "false" ];
then
    GUI=$1
else
    echo "gui argument invalid"
    exit 1
fi

CURDIR=`pwd`
DIR=`dirname $0`

kill_running_nodes(){
# Kill all ros nodes that may be running
for i in $(ps ax | grep -w 'ros' | grep -v 'ros2cli' | grep -v 'vscode' | grep -v 'grep' | awk '{print $1}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep suave_reasoner | grep -v /ros/ | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep 'gz sim' | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep mav | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep ardu | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 15 #let it finish the killing spree
}

run_missions(){
  for j in {1..20}
  do
      kill_running_nodes
      FILENAME="${MANAGER}_${MTYPE}"
      echo $FILENAME
      #should add some geometry to this so they don't stack on top of each other
      xfce4-terminal --execute ./scripts/start_ardusub.sh $GUI
      sleep 10 #let it boot up
      xfce4-terminal --execute ./scripts/launch_sim.sh $GUI
      sleep 30 #let it boot up

      rm -f ~/suave_ws/mission.done
      xfce4-terminal --execute ./scripts/launch_mission.sh $MANAGER $MTYPE $FILENAME
      sleep 30 #let it boot up

      echo "start waiting for mission to finish"
      start_time=$SECONDS
      while [ ! -f ~/suave_ws/mission.done ]
      do
          if [ -f ~/suave_ws/mission.done ]
          then
              echo "mission done"
              rm ~/suave_ws/mission.done
              break;
          fi
          current_time=$SECONDS
          elapsed="$(($current_time-$start_time))"
          if (($elapsed>350))
          then
            echo "mission aborted"
            break;
            sleep 5 #sustainability!
          fi
      done

      echo "killing nodes"
      kill_running_nodes
  done
}

cd ~/ardupilot
./waf configure && make sub
cd $CURDIR

MANAGER="metacontrol"
MTYPE="time"
run_missions

MANAGER="random"
run_missions

MANAGER="none"
run_missions
