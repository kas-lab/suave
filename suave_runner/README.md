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
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=metacontrol\", \
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
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt use_action_server:=true\", \
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
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=random\", \
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
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=none\", \
      \"num_runs\": 6, \
      \"adaptation_manager\": \"none\", \
      \"mission_name\": \"suave\"}"
  ]'
```

## Statistical analysis

The `statistical_analysis` executable compares mission-level metrics produced by
two or more managing systems. For each ordered pair of systems, it reports
Shapiro-Wilk normality diagnostics and performs one-sided Mann-Whitney U tests
for the following alternatives:

| Metric | Alternative for row system A against column system B |
|---|---|
| Time searching for the pipeline | A is lower than B |
| Distance inspected | A is greater than B |
| Mean reaction time | A is lower than B |

Each input CSV file must contain these columns exactly:

- `time searching pipeline (s)`
- `distance inspected (m)`
- `mean reaction time (s)`

These columns are written by `suave_metrics`. If a run does not find the
pipeline, its search time equals its full mission duration. If a run has no
adaptation events, its mean reaction time is `0.0`.

Run the analysis from a sourced SUAVE workspace:

```bash
ros2 run suave_runner statistical_analysis \
  --ros-args \
  -p result_path:=~/suave/results/statistical_analysis \
  -p filename:=none_vs_bt \
  -p data_files:='[
    "{\"managing_system\": \"none\", \
      \"data_file\": \"~/suave/results/none_suave.csv\"}",
    "{\"managing_system\": \"suave_bt\", \
      \"data_file\": \"~/suave/results/bt_suave.csv\"}"
  ]'
```

The ROS parameters are:

| Parameter | Default | Description |
|---|---|---|
| `data_files` | `['']` (placeholder) | Required array of JSON strings containing `managing_system` and `data_file` |
| `result_path` | `~/suave/results` | Directory in which result matrices are written |
| `filename` | `suave_statistical_analysis` | Prefix used for result matrix filenames |

The command prints normality p-values and directional test conclusions using a
significance threshold of `0.05`. The normality results are diagnostic only and
do not alter the Mann-Whitney tests. The command also writes three CSV matrices:

- `<filename>_time_search_pipeline.csv`
- `<filename>_distance_inspected.csv`
- `<filename>_mean_reaction_time.csv`

Matrix rows and columns are managing-system names. Cell `(A, B)` contains the
raw p-value for the directional alternative shown above, and diagonal cells are
empty. The script does not apply a multiple-comparison correction. A p-value at
or above `0.05` means that the analysis did not find sufficient evidence for
the specified direction; it does not establish equality or the opposite
direction. Mann-Whitney U compares sample ranks and distributions rather than
arithmetic means directly.

## Result summary

The `summarize_results` executable produces descriptive statistics for all
managing systems in one campaign folder. It discovers aggregate result files
named `<managing_system>_suave.csv`, such as `bt_suave.csv` and
`metacontrol_suave.csv`. Other CSV files, including per-event reaction-time and
reasoning-time files, are ignored.

Each aggregate result file must contain at least these columns:

- `pipeline found`;
- `time searching pipeline (s)`;
- `distance inspected (m)`; and
- `mean reaction time (s)`.

The `pipeline found` column accepts `true`/`false`, `1`/`0`, or `yes`/`no`
(case-insensitive). Metric columns must contain numeric, non-missing values,
and each file must contain at least one result row.

Run the command from a sourced SUAVE workspace, passing the campaign folder as
the positional argument:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44
```

The terminal table reports the following information for each managing system:

- pipeline detection success as both `successful/total` and a percentage;
- mean and sample standard deviation of the pipeline search time;
- mean and sample standard deviation of the distance inspected; and
- mean and sample standard deviation of the reaction time.

Search time and distance use every run. Reaction-time calculations exclude
zero-valued runs because zero indicates that no reaction-time sample was
recorded. Reaction time is reported as `N/A` if no nonzero samples remain. The
sample standard deviation is also `N/A` when fewer than two applicable samples
are available.

Managing-system identifiers are derived from filenames and formatted for
presentation. The known identifiers are `bt`, `metacontrol`, `none`, `random`,
and `rebetmc`, displayed as `BT`, `Metacontrol`, `None`, `Random`, and
`ReBeT-MC`, respectively. Other identifiers are converted from snake case to
title case.

Use `--latex` to also write a complete LaTeX table to
`<results_path>/suave_results.tex`:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44 \
  --latex
```

Use `--latex-output` to select another output file. Parent directories are
created automatically. Supplying this option also enables LaTeX generation,
so `--latex` is not required:

```bash
ros2 run suave_runner summarize_results \
  ~/suave/results/2026_08_01_19-42-44 \
  --latex-output ~/paper/tables/suave_results.tex
```

The available arguments are:

| Argument | Default | Description |
|---|---|---|
| `results_path` | Required | Campaign folder containing the aggregate `*_suave.csv` files |
| `--latex` | Disabled | Write `<results_path>/suave_results.tex` |
| `--latex-output PATH` | Not set | Write the table to `PATH`; also enables LaTeX output |
| `--precision N` | `2` | Use `N` decimal places in terminal and LaTeX output |

The LaTeX output is a complete `table` environment with the caption
`SUAVE results mean and standard deviation.` and label `tab:suave_results`.
It uses `\toprule`, `\midrule`, and `\bottomrule` from `booktabs` and
`\resizebox` from `graphicx`, so include both packages in the parent document:

```latex
\usepackage{booktabs}
\usepackage{graphicx}
```

The generated file can then be included with `\input`, for example:

```latex
\input{tables/suave_results}
```
