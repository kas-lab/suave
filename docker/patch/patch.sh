#!/bin/bash

cd ~/suave_ws/
rm -rf build/ install/ log/

cd ~/suave_ws/src/remaro_worlds && git pull
cd ~/suave_ws/src/mavros_wrapper && git pull
cd ~/suave_ws/src/mc_mros_reasoner && git pull
cd ~/suave_ws/src/bluerov2_ignition && git pull
cd ~/suave_ws/src/suave && git pull
cd ~/suave_ws && colcon build --symlink-install
cd ~/suave_ws/src/suave/docker/runner

echo "patch"
