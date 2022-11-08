To run lawn_mower:

In first terminal run: 

sim_vehicle.py -v ArduSub -L RATBeach --out=udp:0.0.0.0:14550 --map --console

In second terminal run:

cd pipeline_inspection

source install/setup.zsh or setup.bash

ros2 launch pipeline_inspection pipeline_env.launch.py

In third terminal run lawn_mower.py:

python3 scripts/lawn_mower.py
