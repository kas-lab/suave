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
}


for j in 1 2 3
do
    xfce4-terminal --execute './start_ardusub.sh'
    xfce4-terminal --execute './scripts/launch_sim.sh'
    xfce4-terminal --execute './scripts/launch_mc.sh'
    xfce4-terminal --execute './scripts/launch_mission.sh'
    sleep 30
    kill_running_nodes
done
