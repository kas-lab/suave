# suave_runner

Build the headless image from the repository root:

```bash
docker build -t suave-headless:dev -f docker/dockerfile-suave-headless .
```

Run the local image with NVIDIA GPU support:

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro suave-headless:dev
```

```bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro -v $HOME/ros_workspaces/suave_rebetmc_ws/src/suave:/home/ubuntu-user/suave_ws/src/suave suave-headless:dev
```

The published image is available as `ghcr.io/kas-lab/suave-headless:main`.

```Bash
docker run -it --rm --gpus all --runtime=nvidia --name suave_runner -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -v $HOME/suave/results:/home/ubuntu-user/suave/results -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/localtime:/etc/localtime:ro ghcr.io/kas-lab/suave-headless:main
```

## Resuming a crashed campaign

If the runner crashes or is interrupted mid-campaign, it can resume from where it
left off without re-running completed runs.

After each successful run the runner writes a marker file
`run_<exp_idx>_<run_idx>.done` inside the result folder. On resume the runner
reads these markers and skips any run whose marker already exists. Runs that
timed out (no `mission_metrics/done` received) are **not** marked and will be
retried.

To resume, pass the path of the existing result folder via `resume_result_path`:

```bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p resume_result_path:=~/suave/results/2026_06_19_10-30-00 \
  -p experiments:='[...]'
```

The `experiments` parameter must match the original campaign. Per-run mission
config files (`mission_config_run*.yaml`) are reused if already present in the
folder; otherwise they are regenerated from the same random seed.

## Reproducible perturbations

The runner uses the `random_seed` ROS parameter when generating experiment
perturbations. It defaults to `100` so benchmark runs are reproducible. Set a
different integer in `config/runner_config.yml` or through `--ros-args -p
random_seed:=<value>` to generate a different sequence.


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

Append `use_action_server:=true` to `experiment_launch` to have the BT invoke
managed behaviors through ROS 2 actions:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py use_action_server:=true\", \
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
