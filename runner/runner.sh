#!/bin/bash

if [ $# -lt 1 ]; then
	echo "usage: $0 gui adaptation_manager mission_type runs"
	echo example:
	echo "  "$0 "[true | false] [metacontrol | random | none] [time | distance] runs(integer)"
	exit 1
fi

trap ctrl_c INT

function ctrl_c() {
        echo "Cleaning up.."
        kill_running_nodes
        echo "Shutting down"
        exit 0
}

GUI=""
MANAGER=""
MTYPE=""
FILENAME=""
NOW=""

if [ "$1" == "true" ] || [ "$1" == "false" ];
then
    GUI=$1
else
    echo "gui argument invalid"
    exit 1
fi

if [ "$2" == "metacontrol" ] || [ "$2" == "random" ] || [[ "$2" == "none" ]];
then
    MANAGER=$2
else
    echo "adaptation_manager invalid or missing"
    exit 1
fi

if [ "$3" == "time" ] || [ "$3" == "distance" ];
then
    MTYPE=$3
else
    echo "mission_type invalid or missing"
    exit 1
fi

if [ "$4" != "" ];
then
    RUNS=$4
else
    echo "number of runs invalid or missing"
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
  for ((j=0; j < $RUNS; j++));
  do
      echo "starting run"
      kill_running_nodes
      ros2 daemon stop
      ros2 daemon start
      FILENAME="${MANAGER}_${MTYPE}_${NOW}"
      echo $FILENAME
      #should add some geometry to this so they don't stack on top of each other
      xfce4-terminal --execute ./scripts/start_ardusub.sh $GUI &
      sleep 10 #let it boot up
      xfce4-terminal --execute ./scripts/launch_sim.sh $GUI &
      sleep 30 #let it boot up

      rm -f ~/suave_ws/mission.done
      xfce4-terminal --execute ./scripts/launch_mission.sh $MANAGER $MTYPE $FILENAME &
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
          if (($elapsed>600))
          then
            echo "mission aborted"
            break;
          fi
          sleep 5 #sustainability!
      done

      echo "killing nodes"
      kill_running_nodes
  done
}

cd ~/ardupilot
./waf configure && make sub
cd $CURDIR

NOW=$(date +"%d_%m_%y_%H_%M_%S")
MISSIONCONFIG=${NOW}"_mission_config.yaml"
mkdir -p ~/suave/results
cp ~/suave_ws/install/suave_missions/share/suave_missions/config/mission_config.yaml ~/suave/results/${MISSIONCONFIG}

run_missions
