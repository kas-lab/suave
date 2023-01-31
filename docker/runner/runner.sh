#!/bin/bash

CURDIR=`pwd`
DIR=`dirname $0`

kill_running_nodes(){
# Kill all ros nodes that may be running
for i in $(ps ax | grep ros | grep -v vscode | grep -v grep | awk '{print $1}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep reasoner_node | grep -v /ros/ | grep -v grep | awk '{print $2}')
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
}


for j in 1
do
    #should add some geometry to this so they don't stack on top of each other
    xfce4-terminal --execute './scripts/start_ardusub.sh'
    sleep 30 #let it boot up
    xfce4-terminal --execute './scripts/launch_sim.sh'
    sleep 30 #let it boot up
    xfce4-terminal --execute ./scripts/launch_mission.sh none distance

    echo "start waiting for mission to finish"
    while [ ! -f ~/suave_ws/mission.done ]
    do
        if [ -f ~/suave_ws/mission.done ]
        then
            echo "mission done"
            break;
        fi
        sleep 5 #sustainability!
    done

    
    echo "killing nodes"
    kill_running_nodes

    rm ~/suave_ws/mission.done
done
