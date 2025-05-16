# suave_runner

Runners for SUAVE

- [Container Images](#container-images)
- [suave_runner](#suave_runner)

docker run -it --rm --gpus all --runtime=nvidia -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all
-e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/suave_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave_runner


```
xvfb-run -a your_command_here
```

ros2 run suave_runner suave_runner \
  --ros-args \
  -p experiments:='[
    "{\"suave_base_launch\": \"ros2 launch suave suave.launch.py\", \
      \"experiment_launch\": \"ros2 launch suave_missions mission.launch.py adaptation_manager:=metacontrol\", \
      \"num_runs\": 1, \
      \"adaptation_manager\": \"metacontrol\", \
      \"mission_name\": \"suave\"}"
  ]'


### Container Images

| Description | Image:Tag | Default Command |
| --- | --- | -- |
|  |  |  |


## `suave_runner`

### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
|  |  |  |

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
|  |  |  |

### Services

| Service | Type | Description |
| --- | --- | --- |
|  |  |  |

### Actions

| Action | Type | Description |
| --- | --- | --- |
|  |  |  |

### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
|  |  |  |
