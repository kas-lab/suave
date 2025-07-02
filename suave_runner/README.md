# suave_runner

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave_runner
```

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave ghcr.io/kas-lab/suave-headless:latest
```

```bash
docker run -it --rm   --gpus all   --runtime=nvidia   -e DISPLAY=$DISPLAY   -e QT_X11_NO_MITSHM=1   -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v /home/gus/suave_ws/src/suave/:/home/ubuntu-user/suave_ws/src/suave suave_runner
```

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/suave_ws/src/suave/:/home/ubuntu-user/suave_ws/src/suave suave_runner
```


## Metacontrol

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_metacontrol suave_metacontrol.launch.py\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"metacontrol\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Behavior Tree

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py\", \
      \"num_runs\": 20, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Random

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_random suave_random.launch.py\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"random\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## None

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiment_logging:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_none suave_none.launch.py\", \
      \"num_runs\": 6, \
      \"adaptation_manager\": \"none\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Statistical analysis

```bash
ros2 run suave_runner statistical_analysis \
  --ros-args \
  -p data_files:='[
    "{\"managing_system\": \"none\", \
      \"data_file\": \"~/suave/results/none_suave.csv\"}",
    "{\"managing_system\": \"suave_bt\", \
      \"data_file\": \"~/suave/results/bt_suave.csv\"}"
  ]'
```
