# Run SUAVE

## With Runner
### Trying it out!
If you simply want to try out the exemplar, simply enter the following commands in a terminal:
```Bash
cd ~/suave_ws/src/suave/runner/
./example_run.sh
```
Within a couple of minutes, some new terminals should open as well as the Gazebo simulator.
A default mission is executed of inspecting the pipeline with a time limit.
To follow the robot as it progresses along its mission make sure to right click and follow it in the entity tree of Gazebo as shown below:
![BLUEROV Follow](https://github.com/kas-lab/suave/blob/652db0676ec2995c4cc0653ef5de0fc49edd00ac/docker/follow_bluerov.PNG)

**Please note**: It can take a little while for the robot to get moving, it is an issue we are aware of. Once it does get a move on you should see it perform its mission for about 5 minutes.

### Full Runner

There are two ways to run a full experiment campaign.

**Option 1 — Shell runner** (simple, positional arguments):

```Bash
cd ~/suave_ws/src/suave/runner/
./runner.sh false metacontrol time 2
```

The script takes 4 positional arguments:
1. `true` or `false` — whether to show a GUI
2. `metacontrol`, `random`, `none`, or `bt` — adaptation manager
3. `time` or `distance` — mission type
4. Number of runs (integer)

**Option 2 — ROS 2 runner** (config-file driven, recommended for larger campaigns):

First make sure you source the suave workspace:

```Bash
cd ~/suave_ws/
source install/setup.bash
```

Then run:

Without gui:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=False \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

With gui:
```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p gui:=True \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

To run the BT manager through ROS 2 actions, append
`use_action_server:=true` to its experiment command:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt use_action_server:=true\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

You can also use a launch file with a [config file](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner/runner_config.yml) to make it easier to run the experiments:

```Bash
ros2 launch suave_runner suave_runner_launch.py
```

To run SUAVE with different managing subsystems, replace the `experiment_launch` with the proper launch file.

### Runner config reference

`suave_runner/config/runner/runner_config.yml` controls all experiment parameters. The key fields are:

| Parameter | Default | Description |
|---|---|---|
| `result_path` | `~/suave/results` | Directory where CSV result files and per-run logs are written |
| `gui` | `false` | Launch Gazebo with a visible window |
| `experiment_logging` | `false` | Enable extra console output from the runner (ArduPilot and ROS node logs are always captured to `<result_path>/logs/run_<exp>_<run>/` regardless of this flag) |
| `run_duration` | `600` | Maximum seconds to wait for `mission_metrics/done` before timing out a run |
| `mission_config_pkg` | `suave_missions` | ROS package containing the mission config |
| `mission_config_file` | `config/runner_mission_config.yaml` | Path to mission config within that package |
| `ardupilot_executable` | `sim_vehicle.py -L RATBeach -v ArduSub --model=JSON` | Command used to start ArduPilot SITL |
| `suave_simulation` | `ros2 launch suave simulation.launch.py` | Simulation launch command |
| `initial_pos_x` / `y` / `z` | `-17.0` / `2.5` / `-18.5` | AUV spawn position in the Gazebo world; Y comes from the shipped runner config |
| `initial_pos_x_random_interval` | `[0.0, 0.0]` | Random offset range applied to X spawn position each run |
| `initial_pos_y_random_interval` | `[-0.5, 0.5]` | Random offset range applied to Y spawn position each run |
| `initial_pos_z_random_interval` | `[0.0, 0.0]` | Random offset range applied to Z spawn position each run |
| `water_visibility_sec_shift` | `0.0` | Fixed time offset (s) before the water visibility disturbance |
| `water_visibility_sec_shift_random_interval` | `[0.0, 120.0]` | Random range added on top of the fixed offset |
| `thruster_events` | `[(1,failure,100), (3,failure,100)]` | List of thruster events: `(id, type, time_s)` |
| `thruster_events_random_interval` | `[-100.0, 100.0]` | Random offset (s) applied to each thruster event time |
| `random_interval` | `5` | Number of runs before re-randomising offsets |
| `random_seed` | `100` | Seed for the perturbation RNG — fix this value for reproducible benchmark campaigns |
| `resume_result_path` | `""` | Path to an existing result folder to resume a crashed campaign (empty = start a new timestamped folder) |
| `experiments` | *(see file)* | List of experiment definitions (see below) |

Each entry in `experiments` is a JSON string with four fields:

```yaml
experiments:
  - |
    {
      "experiment_launch": "ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt",
      "num_runs": 10,
      "adaptation_manager": "bt",
      "mission_name": "suave"
    }
```

| Field | Description |
|---|---|
| `experiment_launch` | Full `ros2 launch` command and arguments for the managing subsystem; add `use_action_server:=true` here for BT action mode |
| `num_runs` | Number of times to repeat this experiment |
| `adaptation_manager` | Label written into result CSV files |
| `mission_name` | Mission label written into result CSV files |

Multiple experiments can be listed and will be run sequentially. See the [Metrics Reference](metrics.md) for details on output files.

### Batch runner

To run several campaigns sequentially in one go, use the generic `run_batch`
node, configured through a YAML manifest (see
[config/runner/batch_campaigns.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner/batch_campaigns.yml)
for a runnable example covering `exp1`-`exp3` and `extended_exp1`-`extended_exp3`
with the `bt`, `metacontrol`, `random`, and `none` managing systems bundled
with this repository; a third-party managing system's own runner config can
be added the same way, from its own repo):

```Bash
ros2 launch suave_runner run_batch_launch.py
```

Or directly, pointing at any manifest:

```Bash
ros2 run suave_runner run_batch \
  --ros-args --params-file config/runner/batch_campaigns.yml
```

**Note:** running every campaign back-to-back can take several hours to a few
days, depending on the machine and `num_runs` per experiment -- plan to leave
it running unattended (e.g. in `screen`/`tmux`) rather than waiting on it.

This creates a timestamped batch directory under `~/suave/results/batches/`
(e.g. `batch_20260913_104200/`) containing:

- `state.json` -- the batch checkpoint, tracking each campaign's status
  (`pending`, `running`, `completed`, `incomplete`, `failed`, or
  `interrupted`)
- `run_batch.log` -- the overall orchestration log
- `<campaign_name>.log` -- combined stdout/stderr for each campaign (e.g.
  `exp1.log`)
- `campaigns/<campaign_name>/` -- each campaign's own results directory, in
  the same layout `suave_runner` produces for a single campaign

**Checking whether a batch needs to be resumed:** a batch finished cleanly if
`run_batch.log` ends with `All campaigns completed successfully.`. If it
stops early -- Ctrl+C, a crash, a failed campaign -- the log ends instead with
a `Batch interrupted; ...` or `Batch finished with N incomplete, ...` line
followed by the exact resume command to use. You can also check
campaign-by-campaign at any time by inspecting `state.json`:

```Bash
grep '"status"' ~/suave/results/batches/<batch_dir>/state.json
```

Any campaign not marked `"status": "completed"` still needs to (re)run.

**Resuming an interrupted batch:** pass the batch's `state.json` as the
`resume_state_file` parameter; campaigns already marked `completed` are
skipped, and an `incomplete` campaign picks up from its last saved run
instead of starting over:

```Bash
ros2 run suave_runner run_batch --ros-args \
  -p resume_state_file:=~/suave/results/batches/<batch_dir>/state.json
```

To run a custom subset or order of campaigns, write your own manifest (or
edit `config/runner/batch_campaigns.yml`) and pass it via `--params-file` before
starting a new batch. `-p fail_fast:=true` stops the batch as soon as a
campaign fails instead of continuing to the next one, and `-p dry_run:=true`
prints the `ros2 run` command for each campaign without executing anything.

### Resuming a crashed campaign

If the runner is interrupted mid-campaign (crash, Ctrl+C, power loss), it can
resume from where it left off without re-running completed runs.

After each **successful** run the runner writes a marker file
`run_<exp_idx>_<run_idx>.done` inside the result folder. Runs that timed out
(no result received within `run_duration`) are **not** marked and will be
retried on resume.

If any launched process exits with a nonzero status before mission completion,
or a subsystem publishes `mission/control_failure`, the runner terminates that
run immediately and continues with the remaining runs. The failed run receives
no `.done` marker. Its launch events, process output, and ROS logs remain under
`logs/run_<exp_idx>_<run_idx>/`. The `experiment_logging` flag controls console
verbosity only; it does not disable these files.

To resume, pass the path of the interrupted result folder via
`resume_result_path`:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p resume_result_path:=~/suave/results/2026_06_19_10-30-00 \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt\", \
      \"num_runs\": 10, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

The `experiments` parameter must match the original campaign exactly so that
run indices line up with the marker files. Per-run mission config files
(`mission_config_run*.yaml`) are reused from the existing folder if present;
otherwise they are regenerated using the same random seed.

### Wilcoxon analysis (paired)

The `wilcoxon_analysis` executable implements
a paired Wilcoxon signed-rank test on matched runs across managing systems,
instead of treating them as independent samples. There is a skill for agents for 
performing the `wilcoxon-analysis`, check the skill for the full workflow. Summary:

**Prerequisite: sort first.** Every input CSV must carry a `run_idx` column
identifying which run of that managing system each row is. A `--resume`d
campaign appends retried runs out of order, so this cannot be assumed from
row position. Recover it first with `sort_results.py` (see the
`sort-suave-results` skill):

```bash
python3 suave_runner/analysis/sort_results.py \
  --csv ~/suave/results/bt_suave.csv --exp-idx 2 \
  --output ~/suave/results/sorted/bt_suave_sorted.csv
```

Then run the paired analysis on the sorted CSVs:

```bash
ros2 run suave_runner wilcoxon_analysis \
  --ros-args \
  -p result_path:=~/suave/results/wilcoxon_analysis \
  -p filename:=none_vs_bt \
  -p correction:=holm \
  -p data_files:='[
    "{\"managing_system\": \"none\", \
      \"data_file\": \"~/suave/results/sorted/none_suave_sorted.csv\"}",
    "{\"managing_system\": \"bt\", \
      \"data_file\": \"~/suave/results/sorted/bt_suave_sorted.csv\"}"
  ]'
```

For `exp1`-`exp3` and `extended_exp1`-`extended_exp3` (the campaigns bundled
with this repo's own runner configs), a preconfigured launch file under
`suave_runner/launch/analysis/` avoids typing out `data_files` by hand:

```bash
ros2 launch suave_runner exp1_analysis_launch.py \
  results_root:=~/suave/results/sorted
```

`results_root` must point at that campaign's sorted-CSV directory;
`output_root` (defaults to `results_root`) and `correction` (defaults to
`holm`) can also be overridden. See
[config/analysis/exp1_analysis_config.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/analysis/exp1_analysis_config.yml)
for the exact `data_files` mapping used -- edit a copy of it for a campaign
whose managing systems differ from what's bundled.

Parameters beyond those shared with `mann_whitney_analysis`:

| Parameter | Default | Description |
|---|---|---|
| `correction` | `holm` | Multiple-comparison correction across every pair and metric in the run; set to any other value to disable (raw p-values are then copied into `p_adjusted`) |

Unlike `mann_whitney_analysis`, `data_files` here must point at
`run_idx`-tagged, sorted CSVs. A missing `run_idx` column, a duplicate
`run_idx`, or an unmatched run raises rather than falling back to row order.

The command writes:

- `<filename>_results.csv` -- the primary output, one row per ordered system
  pair and metric (`search_time` and `distance_inspected`), with the
  alternative hypothesis, pair counts
  (declared / used / positive / negative / zero), median paired difference,
  test statistic, raw and Holm-adjusted p-values, a matched rank-biserial
  effect size, and the run indices excluded at each filtering step (missing
  counterpart, pipeline not found for either metric);
- `<filename>_time_search_pipeline.csv` and
  `<filename>_distance_inspected.csv` -- secondary compatibility matrices in
  the same shape `mann_whitney_analysis` writes (raw p-values, not
  Holm-adjusted), so existing table-generation tooling can point at either.

Both `search_time` and `distance_inspected` exclude any pair where either
system did not find the pipeline. Search time is right-censored at the
mission duration in that case, and inspection requires finding the pipeline.
Both comparisons are therefore conditional on detection by both systems.
Holm correction applies across the ordered system pairs for these two metrics.

### Mann-Whitney analysis

The `mann_whitney_analysis` executable
compares mission-level metrics produced by two or more managing systems,
treating each system's runs as independent samples. For each ordered pair of
systems, it reports Shapiro-Wilk normality diagnostics and performs one-sided
Mann-Whitney U tests for the following alternatives:

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
ros2 run suave_runner mann_whitney_analysis \
  --ros-args \
  -p result_path:=~/suave/results/mann_whitney_analysis \
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

Mann-Whitney treats each system's runs as unrelated samples. If the campaign
reused the same perturbation configuration across managing systems (SUAVE
campaigns do, see `sort-suave-results`), use the paired Wilcoxon analysis
below instead -- it controls for per-scenario difficulty and is more
statistically powerful, but requires a run-identity column that a resumed
campaign's raw CSVs do not carry by default.

### Result summary

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
displayed as `BT`, `Metacontrol`, `None`, and `Random`, respectively.
Other identifiers -- including third-party managing systems from other
repos -- are converted from snake case to
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

## Without the runner

**Configuring SUAVE:**
SUAVE has a number of parameters that may be of interest when running experiments with its missions, such as the time limit of a time constrained mission or the frequency of thruster failure. These can be found in the [mission_config.yaml](https://github.com/kas-lab/suave/blob/main/suave_missions/config/mission_config.yaml) file. **Note:** When you change the mission_config file, you need to rebuild the suave_ws with `colcon build --symlink-install`

**Note:** Before starting the simulation or the ros nodes, remember that you have to source SUAVE's workspace. If you are using the dockerized version this is already done for you, therefore sourcing the workspace is not necessary.

Navigate to the workspace and source it:
```Bash
cd ~/suave_ws/
source install/setup.bash
```

With SUAVE configured and sourced, start ArduSub, the simulation, and the SUAVE's nodes with the following instructions.

### Start ArduSub

Run:
```Bash
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

### Start the simulation

Run:
```Bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

**Note:** It is possible to pass arguments to specify the x and y coordinates of where the UUV spawns, by changing the values. In the above launch command the initial coordinates are set to (-17.0, 2.0).

### Start SUAVE's nodes

Run:
```Bash
ros2 launch suave_bringup mission.launch.py
```

**Mission results:** The mission results will be saved in the path specified in the [mission_config.yaml](https://github.com/kas-lab/suave/blob/main/suave_missions/config/mission_config.yaml) file.

**Selecting the managing system and mission type:**
Launching the mission file without launch arguments will start a time-constrained mission without a managing subsystem. To select a different managing subsystem or a different type of mission, the following launch arguments can be used:

```
'adaptation_manager':
    Managing subsystem to be used
    available values: none/metacontrol/random/bt
    (default: 'none')

'result_filename':
    Filename for the mission measured metrics
    available values: any name
    (default: empty; the metrics node uses its default filename)

'mission_type':
    Mission label written to metrics
    (default: 'time_constrained_mission')

'use_action_server':
    For the BT manager, start managed behaviors through ROS 2 actions
    available values: true/false
    (default: false)
```

The arguments can be defined by adding the above arguments with the notation `<name>:=<value>` to the end of the command line.

An example of running a mission with metacontrol saving to a file called 'measurement_1':

```Bash
ros2 launch suave_bringup mission.launch.py adaptation_manager:=metacontrol result_filename:=measurement_1
```

An example of running the BT manager in action-server mode:

```Bash
ros2 launch suave_bringup mission.launch.py \
  adaptation_manager:=bt use_action_server:=true
```


## Q-Q plots of paired differences

`suave_runner/suave_runner/analysis/qq_plot.py` is a standalone Python script using
pandas, NumPy, SciPy, and Matplotlib. It plots A minus B for visual assessment
of the paired t-test normality assumption. It does not run formal normality
tests or classify the data. The reference line is fitted to the differences.

Run these examples from the sourced SUAVE workspace root inside the development
container. Paths must be visible inside that container.

Compare two method columns in one CSV (paired by row):

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot.py results.csv \
  --column-a suave --column-b baseline --output qq_suave_baseline.png
```

Compare the same metric in two sorted method CSVs:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot.py \
  /path/to/campaign/sorted/metacontrol_suave_sorted.csv \
  /path/to/campaign/sorted/bt_suave_sorted.csv \
  --column-a 'time searching pipeline (s)' \
  --column-b 'time searching pipeline (s)' --output qq_search.png
```

Separate files must belong to the same campaign and have matching, unique,
nonmissing `run_idx` values; row order does not matter. `--run-column` selects
another pairing key. Unmatched or duplicate IDs cause an error. The caller
must ensure those IDs refer to the same experimental configurations.
`--label-a` and `--label-b` override method labels derived from filenames.

Use the companion `qq_plot_batch.py` to generate every unordered method
comparison within every campaign. Pass the batch root directly:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot_batch.py \
  /home/ubuntu-user/suave/results/batches/all_experiments_20260904_093305
```

The batch script reuses the CSV script's validation and plotting functions;
keep both scripts together. It accepts a batch root, its `campaigns` directory,
or one campaign. It reads only `*_sorted.csv` files from each campaign's
`sorted` folder and never pairs different campaigns. Search time and distance
inspected are the default metrics; `--metrics` accepts an explicit list of
metric column names.

By default, results mirror the experiment folders under
`<batch>/campaings_results/` (using this exact directory spelling):

```text
all_experiments_20260904_093305/
  campaigns/
    exp1/sorted/*.csv
    exp2/sorted/*.csv
    ...
  campaings_results/
    exp1/q-q-plots/*.png
    exp2/q-q-plots/*.png
    ...
```

`--output /path/to/qq_plots` overrides the batch output root; figures still
go under `<output>/<experiment>/q-q-plots/`. The option
`--format pdf|png|svg` selects the format (default PNG). The batch script
always saves figures and requires no display. For explicit CSV comparisons,
`qq_plot.py --output` selects a figure filename; omitting it displays the plot
interactively. Run either script with `--help` for its full interface.

Both selected columns must be numeric. A missing value removes the entire
pair. Infinite values are rejected, and at least three complete pairs are
required. All other complete pairs are included, including zero metrics and
runs where `pipeline found` is false. This script does not apply the success
filter used by the Wilcoxon analysis. It prints the number of valid pairs,
removed pairs, mean, median, sample standard deviation (`ddof=1`), minimum,
and maximum. Batch errors identify the campaign, files, and metric; remaining
comparisons continue, with a nonzero exit status if any comparison fails.


## Batch paired Wilcoxon analysis

`wilcoxon_analysis_batch.py` orchestrates the existing `wilcoxon_analysis.py`
node. Run it from the sourced SUAVE workspace root inside the container:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py \
  /home/ubuntu-user/suave/results/batches/all_experiments_20260904_093305
```

It first sorts the raw aggregate CSVs in each `campaigns/<experiment>/`
using the original `run_<exp_idx>_<run_idx>.done` markers, then analyzes
the verified copies in `sorted/` and writes:

```text
campaings_results/
  exp1/
    q-q-plots/
    wilcoxon_analysis/
      exp1_wilcoxon_results.csv
      exp1_wilcoxon_time_search_pipeline.csv
      exp1_wilcoxon_distance_inspected.csv
  exp2/
    wilcoxon_analysis/
      ...
```

`--output PATH` overrides `campaings_results`, preserving the
`<experiment>/wilcoxon_analysis/` structure. Keep this script alongside
`wilcoxon_analysis.py` and `sort_results.py`. No package rebuild is needed
for this direct invocation.

Each campaign's runner configuration is resolved from `state.json`, supporting
both the original batch launch-name keys and the newer campaign-name keys.
The configuration's `experiments` list determines the method-to-`exp_idx`
mapping, filenames, and expected run counts. If recorded config paths have
moved, pass `--config-dir /path/to/original/configs`; this directory must
contain the recorded config basenames (or `<experiment>_runner_config.yml`
when no config is recorded). Use configs that match the original campaign.
The script does not infer marker indices from alphabetical method order.

The pre-step calls the existing `sort_results.reconstruct_run_index` without
relaxing its count, timestamp-gap, or run-ID checks. Original CSVs and marker
modification times are preserved. Run in the timezone used when the CSV
`datetime` values were recorded. All configured methods are reconstructed and
validated before any sorted CSV is updated; unchanged verified sorted files
retain their modification times, and stale ones are refreshed. Only methods
listed in that campaign configuration are analyzed, so unrelated old files
in `sorted/` are ignored. Missing markers, configs, or raw CSVs, timestamp
mismatches, and incomplete configured run counts fail that campaign before
analysis instead of falling back to possibly stale sorted data.

Runs must share experimental configurations within each campaign; never
match unrelated campaigns just because their numeric IDs agree. The
batch wrapper rejects missing or duplicate IDs, noninteger or negative IDs,
unmatched run-key sets, missing/nonfinite/nonnumeric metric values, and
invalid or missing `pipeline found` status before testing that campaign.
It reports failed campaigns, continues the others, and returns a nonzero
exit status if any campaign fails.

The existing statistical design is preserved: all ordered method pairs are
tested with A minus B, using `less` for search time and `greater` for distance
inspected. Both metrics exclude a pair if either method failed to find the
pipeline. Zero distance is retained when both methods found it. This differs
from the Q-Q script, which includes every complete numeric pair.

Holm correction pools all computed ordered-pair tests across both metrics
**within each campaign**, separately from the other campaigns. The default is
`--correction holm`; `--correction none` explicitly disables adjustment.

The primary `*_results.csv` reports raw and adjusted p-values, paired counts,
positive/negative/zero differences, median difference, rank-biserial effect
size, exclusions, and notes. Fewer than two nonzero differences produces an
uncomputed test with a note and missing p-values. `n_used` includes zeros;
`n_pos + n_neg` is the effective nonzero count. Negative effects favor A for
search time; positive effects favor A for distance. The secondary matrices
and the existing engine's console significance labels use **raw** p-values;
use `p_adjusted` in the primary CSV when interpreting corrected results.
A nonsignificant result does not establish equality. Independent pairs and
reasonably symmetric paired differences are needed for the usual
location-shift interpretation.


## Batch Wilcoxon LaTeX tables

Generate one table per experiment from an existing batch analysis, using the
existing `latex_tables.py` renderer. From the sourced container workspace:

```bash
python3 src/suave/suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py \
  /home/ubuntu-user/suave/results/batches/all_experiments_20260904_093305
```

The wrapper discovers experiments under `<batch>/campaigns/` and reads each
`<batch>/campaings_results/<experiment>/wilcoxon_analysis/*_results.csv`.
It saves tables directly under the requested common directory:

```text
campaigns_latex_tables/wilcoxon_analysis/
  exp1_wilcoxon.tex
  exp2_wilcoxon.tex
  exp3_wilcoxon.tex
  extended_exp1_wilcoxon.tex
  extended_exp2_wilcoxon.tex
  extended_exp3_wilcoxon.tex
```

Each table contains search-time and distance-inspected blocks. It defaults
to `p_adjusted` from the primary results CSV; it does not use the secondary
raw-p-value matrices or recompute correction. Temporary matrices adapt those
values to the existing renderer and are removed after generation. Run
`wilcoxon_analysis_batch.py` first if analysis is missing; table generation
does not rerun sorting or statistical tests.

| Option | Meaning |
|---|---|
| `--results-root PATH` | Override `<batch>/campaings_results` |
| `--output PATH` | Override the directory containing all generated tables |
| `--p-values adjusted\|raw` | Display adjusted values by default; raw requires explicit selection |
| `--alpha VALUE` | Color threshold, default 0.05 |

Captions identify the p-value policy, one-sided alternatives, and the condition
that both methods found the pipeline. Green cells indicate values below
alpha; blue cells indicate other computed values. Colors use unrounded
p-values, while printed values retain the renderer's three-decimal format.
Values that would round to `0.000` are displayed as `<0.001`.
A dash denotes a diagonal entry or a test not computed; details remain in
the primary analysis CSV. The wrapper checks both metric blocks for complete,
unique ordered pairs, consistent alternatives and correction policy, valid
p-values, and notes explaining missing results.

Known method labels use None, Random, BT, MC, ROSA, and PLANTA; captions
explain that MC means Metacontrol. ROSA and PLANTA are labels for two
third-party managing systems, each from its own separate repo (not bundled
with this one) -- they only appear when their result CSVs are added
alongside this repo's own for a combined comparison. Tables follow the
paper's pairwise-table
layout: a 1.65 cm row-label column and 1.2 row spacing. Value columns are
slightly wider at 1.3 cm so the names fit in the manuscript's font.
Tables with up to three methods use `0.7\textwidth`; larger tables use
`\textwidth`. Rerun the batch command to refresh existing `.tex` files.
Other labels are escaped as literal LaTeX text. Table labels include the
experiment name. Keep `latex_tables.py` alongside the batch script.

The containing LaTeX document needs the following preamble support (reuse
existing definitions when present):

```latex
\usepackage{array,graphicx,multirow}
\usepackage[table]{xcolor}
\definecolor{pvalue_green}{RGB}{210,245,210}
\definecolor{pvalue_blue}{RGB}{210,225,245}
```

Include a generated table with `\input{path/to/exp1_wilcoxon.tex}`.
Campaign failures are reported individually; other tables are still generated,
and the script exits nonzero if any campaign fails.


## Analysis and LaTeX script locations

The Python tools are grouped under `suave_runner/suave_runner/`:

- `analysis/`: Mann-Whitney and Wilcoxon analysis, batch Wilcoxon analysis,
  result sorting, Q-Q plotting, and descriptive result summaries.
- `latex/`: the table renderer and batch Wilcoxon LaTeX table generation.

`suave_runner.py` and `run_batch.py` remain at the package root for experiment
execution. The installed `mann_whitney_analysis`, `wilcoxon_analysis`, and
`summarize_results` command names are unchanged; their entry points now load
`suave_runner.analysis` modules. Existing launch `executable` names and YAML
node names therefore remain valid. Python callers should import from
`suave_runner.analysis` or `suave_runner.latex`.

After updating the checkout, rebuild inside the sourced SUAVE container:

```bash
colcon build --symlink-install --packages-select suave_runner
source install/setup.bash
```

Direct batch commands, from the workspace root:

```bash
python3 src/suave/suave_runner/suave_runner/analysis/wilcoxon_analysis_batch.py /path/to/batch
python3 src/suave/suave_runner/suave_runner/analysis/qq_plot_batch.py /path/to/batch
python3 src/suave/suave_runner/suave_runner/latex/wilcoxon_latex_tables_batch.py /path/to/batch
```
