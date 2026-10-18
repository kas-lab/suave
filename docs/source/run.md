# Run SUAVE

Use the SUAVE CLI ({doc}`cli`) to run campaigns, sequence batches, diagnose
failures, and resume interrupted experiments. See {doc}`results` for statistical
analysis, summaries, plots, and LaTeX tables.

## Prerequisites

To run SUAVE you either need need to have a install SUAVE deps and build a ROS workspace with SUAVE, or you can use the docker images provided, check the isntalation and docker pages for more info. The easiest way to run SUAVE its through its CLI, which requires that you either run it after sourcing a workspace with SUAVE built, or sourcing `env.sh` directly, or running from a development container with ROS and the workspace installed.
The CLI can invoke that container from the host. Put `suave` on `PATH` by
sourcing the checkout's `env.sh`, or use the workspace's installed CLI hook.
See {doc}`cli` for setup and execution-target settings.

If the container is not named `suave`, pass `--container NAME`. Use paths visible
inside that container. The CLI does not translate host paths, and relative
configuration paths resolve from the container's workspace root.

Run direct ROS commands inside the container. If the environment does not
already source the workspace, run:

```bash
cd ~/suave_ws
source install/setup.bash
```

After changing installed configuration files, rebuild the package that owns
them, for example `suave build suave_runner` or `suave build suave_missions`.

## Run a campaign

A campaign runs one or more experiment definitions sequentially. Each definition
selects a managing system and a number of repeated runs. The runner starts and
stops SITL, simulation, and mission processes for each run.

```bash
suave run
suave run --config src/suave/my_runner.yml
```

The first command uses the installed
[runner_config.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner/runner_config.yml).
The second uses a custom ROS parameter file. Common overrides are:

```bash
suave run --config src/suave/my_runner.yml --gui --seed 100 --run-duration 600
suave run --config src/suave/my_runner.yml -p record_bags:=false
```

`--gui` enables the Gazebo window. To disable it explicitly, pass `-p gui:=false`.
`--run-duration` limits the wait for mission completion, not the entire startup
and shutdown sequence.

### Configuration example

Save this file as `my_runner.yml` in the checkout:

```yaml
/suave_runner_node:
  ros__parameters:
    gui: false
    random_seed: 100
    record_bags: true
    mission_config_pkg: suave_missions
    mission_config_file: config/runner_mission_config.yaml
    experiments:
      - |
        {
          "experiment_launch": "ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt",
          "num_runs": 2,
          "adaptation_manager": "bt",
          "mission_name": "suave",
          "required_services": ["/f_maintain_motion/change_mode", "/f_generate_search_path/change_mode", "/f_follow_pipeline/change_mode"]
        }
```

Append experiment entries to compare more systems. For an external manager,
replace `experiment_launch` with its launch command. For BT action mode, append
`use_action_server:=true` to that command. An individual experiment can override
campaign recording with `record_bags:=false` or `record_bags:=true` in the same
command.

### Direct ROS alternatives

Use the runner launch file with its installed configuration or a custom path:

```bash
ros2 launch suave_runner suave_runner_launch.py
ros2 launch suave_runner suave_runner_launch.py \
  config_file:=/home/ubuntu-user/suave_ws/src/suave/my_runner.yml
```

To override individual node parameters, run the node directly:

```bash
ros2 run suave_runner suave_runner --ros-args \
  --params-file src/suave/my_runner.yml -p gui:=false
```

## Run a batch

A batch runs several campaigns sequentially. Each campaign references a runner
configuration installed by a ROS package. Use the bundled manifest or a custom
one:

```bash
suave batch start
suave batch start --config src/suave/my_batch.yml --fail-fast
suave batch start --config src/suave/my_batch.yml --dry-run-batch
```

`--dry-run-batch` prints campaign commands without executing them or creating
batch files. It does not check installed configuration files.
The CLI's global `--dry-run` instead previews the CLI's wrapped command.
`--fail-fast` stops when a campaign process cannot start or exits nonzero.
An incomplete campaign that exits zero does not trigger this condition.

Example `my_batch.yml`:

```yaml
/run_batch_node:
  ros__parameters:
    campaigns:
      - '{"name": "exp1", "package": "suave_runner", "config_file": "config/runner/exp1_runner_config.yml"}'
      - '{"name": "exp2", "package": "suave_runner", "config_file": "config/runner/exp2_runner_config.yml"}'
```

Campaign names must be unique. List entries in execution order. Third-party
packages can supply their own installed configurations through the same fields.
See the bundled
[batch_campaigns.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner/batch_campaigns.yml).
Batches can take hours or days. Use `screen` or `tmux` for unattended execution.

Direct ROS alternatives:

```bash
ros2 launch suave_runner run_batch_launch.py
ros2 run suave_runner run_batch --ros-args \
  --params-file src/suave/my_batch.yml -p fail_fast:=true
```

The batch launch file always loads the installed `batch_campaigns.yml` and
declares no configuration-path argument. Use the node or CLI for custom files.

## Monitor and diagnose

```bash
suave campaign list
suave batch list
suave batch diagnose --latest
suave batch diagnose /home/ubuntu-user/suave/results/batches/BATCH --json
```

`diagnose` accepts a batch directory, its `state.json`, or a campaign directory.
It only reads files. It reports completed and expected run counts, failures by
manager, failure reasons, attempt log paths, and log lines that suggest a cause.
The direct equivalent is `ros2 run suave_runner diagnose_batch PATH [--json]`.

The runner prints per-manager summaries after each run. The batch runner prints
them after each campaign. A batch completes when all configured runs have
`status: completed` in their campaign's `runs.json`. The normal success message
is `All campaigns completed successfully.` An unfinished or interrupted batch
normally prints a resume command. Early startup errors can stop it before that
summary.

| Failure type | Meaning |
|---|---|
| `mission_done_timeout` | No `mission_metrics/done` within `run_duration` |
| `process_died_before_completion` | A launched process exited nonzero |
| `mission_control_failure` | A subsystem published `mission/control_failure`, including lifecycle-watchdog activation failures |
| `readiness_timeout` | SITL or simulation did not become ready before its timeout |
| `readiness_missing_service` | A required experiment service did not appear before `readiness_timeout` |
| `startup_failure` | Starting the run raised an exception |
| `recording_invalid` | The completed attempt's recording failed validation |
| `shutdown_unconfirmed` | The runner could not confirm process shutdown and stopped the campaign |
| `runner_exception` | The runner raised an exception during the mission stage |
| `interrupted` | Termination interrupted the run |
| `unknown` | The diagnostic summary has no recorded failure type |

A `SYSTEMIC: <manager>: repeated <failure_type>` warning identifies the same
failure in at least three runs, or in at least two runs and more than half of
that manager's attempted runs. Inspect one failed attempt's logs and correct
configuration, dependency, or runtime problems before retrying.

## Resume

### Batch

```bash
suave batch resume --latest
suave batch resume /home/ubuntu-user/suave/results/batches/BATCH/state.json
```

The batch skips completed campaigns and resumes unfinished ones from their
`runs.json`. Its `state.json` stores campaign configuration references, not
progress. Keep those installed configurations unchanged when resuming.

Direct ROS equivalent:

```bash
ros2 run suave_runner run_batch --ros-args \
  -p resume_state_file:=/home/ubuntu-user/suave/results/batches/BATCH/state.json
```

### Campaign

```bash
suave campaign resume --latest --config src/suave/my_runner.yml
suave campaign resume /home/ubuntu-user/suave/results/CAMPAIGN \
  --config src/suave/my_runner.yml
```

Reuse the original runner configuration, including experiment order, run counts,
seed, spawn settings, and perturbations. The runner indexes completion by
`run_<experiment_index>_<run_index>` in `runs.json`, not by CSV rows.
Changing the configuration can associate existing checkpoints with different
experiments. Existing `mission_config_run*.yaml` files are reused. Missing
files are regenerated from the supplied configuration and seed.

`suave campaign resume --latest` searches standalone timestamped campaigns,
not campaigns nested inside a batch. Use batch resume for the latter.

Direct ROS equivalent:

```bash
ros2 run suave_runner suave_runner --ros-args \
  --params-file src/suave/my_runner.yml \
  -p resume_result_path:=/home/ubuntu-user/suave/results/CAMPAIGN
```

### Recovery safeguards

Before retrying an unfinished attempt, the runner checks that its recorded
launch session has no live processes. Recovery does not kill those processes.
It then resolves the attempt from its recording location:

| Recording state | Recovery action |
|---|---|
| Accepted recording only | Validate it and mark the run completed, or quarantine it and mark the run failed |
| Pending recording only | Move it to `rosbags/failed/` and mark the run interrupted |
| No recording | Mark the unfinished run interrupted |
| Both pending and accepted locations | Stop recovery with an error |
| Completed run with recording enabled | Revalidate its accepted recording. Stop on a missing or invalid recording rather than rerunning the run |

Before each new attempt, the runner also refuses to start if it detects a live
`gz sim` process or a bound UDP port 9002, used by the vehicle's
`ArduPilotPlugin`. Stop the leftover simulation or SITL process before resuming.
These checks prevent a new run from controlling a stale vehicle.

## Configuration reference

### Runner parameters

These are **node defaults**, not the overrides in the shipped
`runner_config.yml`. Pass overrides through a ROS parameter file or CLI `-p`.

| Parameter | Node default | Description |
|---|---|---|
| `result_path` | `~/suave/results` | Parent directory for new timestamped campaign folders |
| `gui` | `false` | Show the Gazebo window |
| `experiment_logging` | `false` | Enable simulation and experiment console output through their `silent` launch argument. Process and ROS logs remain captured either way |
| `run_duration` | `600` | Maximum seconds to wait for mission completion after startup readiness checks |
| `mission_config_pkg` | `suave_missions` | Package containing the mission configuration |
| `mission_config_file` | `config/mission_config.yaml` | Mission configuration path relative to the package share directory |
| `enable_water_current` | `false` | Enable the optional water-current node |
| `ardupilot_executable` | `sim_vehicle.py -N -L RATBeach -v ArduSub --model=JSON` | SITL command. `-N` disables rebuilding |
| `ardupilot_ready_timeout` | `30.0` | Timeout in seconds for each SITL port-release and startup wait |
| `suave_simulation` | `ros2 launch suave simulation.launch.py` | Simulation launch command |
| `initial_pos_x` / `initial_pos_y` / `initial_pos_z` | `-17.0` / `2.0` / `-18.5` | Spawn position in the Gazebo world |
| `initial_pos_x_random_interval` | `[0.0, 0.0]` | X spawn-offset range |
| `initial_pos_y_random_interval` | `[0.0, 0.0]` | Y spawn-offset range |
| `initial_pos_z_random_interval` | `[0.0, 0.0]` | Z spawn-offset range |
| `water_visibility_sec_shift` | `0.0` | Fixed visibility-disturbance time shift in seconds |
| `water_visibility_sec_shift_random_interval` | `[0.0, 0.0]` | Additional random visibility time-shift range |
| `thruster_events` | `['']` | Thruster-event strings `(id, type, time_s)`. The default specifies no events |
| `thruster_events_random_interval` | `[0.0, 0.0]` | Random time-shift range applied to the thruster events |
| `random_interval` | `5` | Number of run indices sharing offsets before resampling |
| `random_seed` | `100` | Perturbation RNG seed |
| `simulation_ready_timeout` | `30.0` | Seconds to wait for the first `/model/bluerov2/pose` message |
| `readiness_timeout` | `10.0` | Seconds to wait for experiment `required_services` |
| `resume_result_path` | `''` | Existing campaign folder to resume. Empty starts a timestamped folder |
| `experiments` | `['']` | Placeholder. Supply JSON experiment entries as in the configuration example |
| `record_bags` | `true` | Record each attempt. An experiment's launch command can override this setting |

The shipped `runner_config.yml` sets Y to `2.5`, its random range to
`[-0.5, 0.5]`, the visibility-shift range to `[0.0, 120.0]`, and the thruster
shift range to `[-100.0, 100.0]`. It schedules failures of thrusters 1 and 3 at
100 seconds, selects `config/runner_mission_config.yaml`, and defines four
managers with ten runs each. Other campaign configurations can differ.

Offsets remain constant for `random_interval` runs and reuse the same run-index
configuration across managers. Keep the seed and configuration fixed for
reproducible comparisons.

### Experiment fields and startup

| Field | Description |
|---|---|
| `experiment_launch` | Full `ros2 launch` command for the mission and managing subsystem |
| `num_runs` | Number of repetitions |
| `adaptation_manager` | Manager identifier used in result filenames and run metadata |
| `mission_name` | Mission identifier used in result filenames and run metadata |
| `required_services` | Optional absolute service names to await. Omission disables this wait |

Provide all four main fields. The runner starts each attempt in this order,
using readiness checks rather than fixed startup delays:

1. Wait for TCP port 5760 to become free, start SITL, then wait for it to listen
   on that port. Each wait uses `ardupilot_ready_timeout`.
2. Start simulation and wait for `/model/bluerov2/pose`, bounded by
   `simulation_ready_timeout`.
3. Launch the experiment and wait for `required_services`, bounded by
   `readiness_timeout`. Then wait for `mission_metrics/done`.

The bundled BT configurations require the three mode-change services shown in
the example. Extended BT configurations also require
`/generate_recharge_path/change_mode`. List the services required by an external
manager in its experiment entry.

### Batch parameters

| Parameter | Default | Description |
|---|---|---|
| `batch_dir` | `''` | New batch directory, which must not exist. Empty generates `~/suave/results/batches/batch_YYYYMMDD_HHMMSS` |
| `resume_state_file` | `''` | Existing batch manifest to resume. Mutually exclusive with `batch_dir` |
| `fail_fast` | `false` | Stop if a campaign process cannot start or exits nonzero |
| `dry_run` | `false` | Print campaign commands from `campaigns` without executing them. Does not load a resume manifest |
| `campaigns` | `['']` | JSON strings with unique `name`, installed `package`, and package-relative `config_file`. Required for new batches and dry runs |

## Output layout and completion status

Standalone campaigns create `<result_path>/YYYY_MM_DD_HH-MM-SS/`.
Batch campaigns use `<batch_dir>/campaigns/<campaign_name>/` instead.
Each campaign contains:

```text
CAMPAIGN/
  <manager>_<mission>.csv
  mission_config_run<run_index>.yaml
  runs.json
  logs/run_<experiment_index>_<run_index>/<attempt_id>/
    ardupilot.log
    simulation/
    experiment/
  rosbags/
    pending/<manager>_<mission>_<run_id>_<attempt_id>/
    <manager>_<mission>_<run_id>_<attempt_id>/
    failed/<manager>_<mission>_<run_id>_<attempt_id>/
```

The runner writes `runs.json` as the completion checkpoint. Each run records its
configuration, current attempt ID, status, recording state, timestamps, and
failure details. Earlier attempts remain under `attempts`. The campaign header
records the seed, simulation launch command, and SITL command.

A run is complete only when its status is `completed`. Mission completion does
not imply pipeline detection. A failed process or `mission/control_failure`
fails the run. The runner continues after ordinary run failures, but stops if
it cannot establish safe shutdown or detects a stale simulation.

With recording enabled, attempts start under `rosbags/pending/`. After mission
completion and process shutdown, the runner validates MCAP metadata, storage
files, and the reader stream. It moves a valid bag to the accepted location
before writing `status: completed` and `bag: accepted`. Failed or interrupted
attempts move to `rosbags/failed/` when a recording exists. Without recording,
`bag` is `null` and completion does not require bag validation.

A batch also contains `state.json`, `run_batch.log`, and one `<campaign_name>.log`
with combined campaign stdout and stderr. The manifest stores campaign
references. Each campaign's `runs.json` stores progress. See {doc}`metrics` for
CSV contents and {doc}`results` for selecting attempts during analysis.

## Run manually

Start these commands in **three separate terminals** inside the container,
with the workspace environment available in each.

1. Start SITL:

   ```bash
   sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console
   ```

2. Start simulation:

   ```bash
   ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
   ```

3. Start the mission:

   ```bash
   ros2 launch suave_bringup mission.launch.py
   ```

Wait for SITL and simulation to become ready before starting the mission.
The default launch runs a time-constrained mission without an adaptation manager.
To follow the vehicle in Gazebo, right-click its entity and select follow.

| Launch argument | Default | Purpose |
|---|---|---|
| `adaptation_manager` | `none` | Select `none`, `metacontrol`, `random`, or `bt` |
| `mission_config` | Installed `suave_missions/config/mission_config.yaml` | Mission parameters, including the time limit and disturbance settings |
| `mission_type` | `time_constrained_mission` | Metrics label, not a mission-executable selector |
| `result_path` | `~/suave/results` | CSV and recording output directory. Overrides the mission YAML's metrics path |
| `result_filename` | Empty | Metrics filename. Empty uses the metrics node's default |
| `record_bags` | `true` | Enable MCAP recording |
| `use_action_server` | `false` | Run BT-managed behaviors through ROS actions |
| `bt_executable` | `suave_bt` | Select `suave_bt` or `suave_bt_extended` |

For non-BT managers, the launch file starts `time_constrained_mission`.
Changing `mission_type` only changes the metrics label. BT execution controls
its own mission sequence. Inspect all launch arguments with
`ros2 launch suave_bringup mission.launch.py --show-args`.

Examples:

```bash
ros2 launch suave_bringup mission.launch.py \
  adaptation_manager:=metacontrol result_filename:=measurement_1 record_bags:=false
ros2 launch suave_bringup mission.launch.py \
  adaptation_manager:=bt use_action_server:=true
```

## Legacy desktop shell runner

The shell runner requires a GUI desktop and `xfce4-terminal`, even when Gazebo's
GUI is disabled. It uses fixed startup delays and broad process cleanup rather
than the ROS runner's readiness checks and `runs.json` recovery. Run it in a
dedicated container without unrelated ROS or Gazebo processes.

```bash
cd ~/suave_ws/src/suave/runner
./example_run.sh
```

The example runs one time-constrained mission with Metacontrol and the GUI.
For explicit settings:

```bash
./runner.sh false metacontrol time 2
```

The four positional arguments are GUI (`true` or `false`), manager (`none`,
`metacontrol`, `random`, or `bt`), mission label (`time`, `distance`, or
`extended`), and repetition count. The script maps these labels to launch
arguments. The distance label does not select a distance-constrained executable
in the current mission launch file. The `bt extended` combination selects
`suave_bt_extended`.
