# Run SUAVE

> Tip: the `suave` CLI ({doc}`cli`) wraps the runner, batch and resume commands below.
>
> For statistical analysis, result summaries, plots, and LaTeX tables, see
> {doc}`results`.

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

### Runner configuration reference

`suave_runner/config/runner/runner_config.yml` controls all experiment parameters. The key fields are:

| Parameter | Default | Description |
|---|---|---|
| `result_path` | `~/suave/results` | Directory where CSV result files and per-run logs are written |
| `gui` | `false` | Launch Gazebo with a visible window |
| `experiment_logging` | `false` | Enable extra console output from the runner (ArduPilot and ROS node logs are always captured to `<result_path>/logs/run_<exp>_<run>/<attempt_id>/` regardless of this flag) |
| `run_duration` | `600` | Maximum seconds to wait for `mission_metrics/done` before timing out a run |
| `mission_config_pkg` | `suave_missions` | ROS package containing the mission config |
| `mission_config_file` | `config/runner_mission_config.yaml` | Path to mission config within that package |
| `enable_water_current` | `false` | Enable the optional `water_current` node for each run |
| `ardupilot_executable` | `sim_vehicle.py -N -L RATBeach -v ArduSub --model=JSON` | Command used to start ArduPilot SITL (`-N`: no rebuild) |
| `ardupilot_ready_timeout` | `30.0` | Seconds to wait for SITL to listen on port 5760 (and for a previous SITL to release it) |
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
| `simulation_ready_timeout` | `30.0` | Seconds to wait for the first `/model/bluerov2/pose` message after launching the simulation |
| `readiness_timeout` | `10.0` | Seconds to wait for the experiment's `required_services` before failing a run |
| `random_seed` | `100` | Seed for the perturbation RNG — fix this value for reproducible benchmark campaigns |
| `resume_result_path` | `""` | Path to an existing result folder to resume a crashed campaign (empty = start a new timestamped folder) |
| `experiments` | *(see file)* | List of experiment definitions (see below) |
| `record_bags` | `true` | Whether to record ROS bags for the whole campaign. Individual experiments can override it in the `experiment_launch` command, e.g., `ros2 launch suave_bringup mission.launch.py adaptation_manager:=bt record_bags:=true` |

Each entry in `experiments` is a JSON string with four required fields and an optional `required_services` list:

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
| `required_services` | Optional absolute service names the runner waits for before the mission starts (see readiness below), absent means no wait |

Multiple experiments can be listed and will be run sequentially. See the [Metrics Reference](metrics.md) for details on output files.

**Experiment startup procedure:** The runner has no fixed sleeps: each launch stage waits until it is ready,
and the timeouts are ceilings. The runner logs how long each stage took
(`ArduPilot ready after 2.1s`, `Simulation ready after 6.4s`).

1. ArduPilot: the runner first waits for TCP port 5760 to be free,
   then starts `ardupilot_executable` and waits until SITL listens on port 5760, 
   its MAVLink port (they might be blocked by leftover processes). The waiting time is configured via the param
   `ardupilot_ready_timeout` (default `30.0` s).
3. Simulation: after the simulation launch, the runner waits for the first
   message on `/model/bluerov2/pose`, which `ros_gz_bridge` only forwards
   once Gazebo runs with the vehicle spawned. The ceiling is
   `simulation_ready_timeout` (default `30.0` s).
4. Manager services: after the experiment launch, the runner waits up to
   `readiness_timeout` seconds (default `10.0`) for the services listed in
   the experiment's `required_services`. An experiment without
   `required_services` does not wait. The shipped configs list
   `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`,
   and `/f_follow_pipeline/change_mode` for every `bt` experiment, plus
   `/generate_recharge_path/change_mode` when it launches
   `suave_bt_extended`. **If you are adding your custom managing system, it can be useful to customize this parameter to make sure it maps all services required by it**

When a stage that not ready in time, the runner aborts the run and mark it as failed with the code `readiness_timeout`. A
missing service fails it as `readiness_missing_service`. Either way the runner continues with the next run.

### Runner output and behavior reference

The runner stores its results, logs, rosbags (when `record_bags` = `true`), and meta-data in the folder
registered in the `result_path` parameter. The result folder output has the following structure:

- `result_path`/`<year_month_day_time>`/
  - `<adaptation_manager>`_`<mission_name>`.csv : CSV file with the resulting metrics for each managing system
  - `logs`/`run_<exp_idx>_<run_idx>`/
    - `<attempt_id>` : Process stdout, stderr, launch events, and ROS logs
  - `rosbags`/
    - `pending`/ : rosbags that correspond to runs that were not yet classified as successful or failed
    - `<adaptation_manager>_<mission>_<run_id>_<attempt_id>/`: bags from runs that were considered successful
    - `failed`/`<adaptation_manager>_<mission>_<run_id>_<attempt_id>/`: bags from runs that were considered as failed
  - `runs.json` : meta-data about the runs

Each campaign folder has one `runs.json` file which is written only by
the runner. Its `campaign` header records `random_seed`, `simulation_launch`, and
`ardupilot_command` once. It contains a `runs` field which records each run 
`run_<exp_idx>_<run_idx>` configuration and status, i.e., the `adaptation_manager`, 
`mission_name`, `experiment_launch`, `status` (`running`, `completed`, `failed`, or `interrupted`), `attempt_id`, `bag` (`null` when recording is disabled, otherwise `pending`, `accepted`, or
`failed`), `failure_type`, `failure_reason`, `started_at`, and `finished_at`. 
When a run fails and is later retried, its earlier attempts configuration and status
are stored under `attempts`. A run is complete if and only if its `status` is `completed`.

Example of a run that had one failure and got rerun once and succeeded:

```json
"run_1_27": {
  "adaptation_manager": "rosa_bt",
  "mission_name": "suave",
  "experiment_launch": "ros2 launch suave_rosa_bt suave_rosa_bt.launch.py",
  "status": "completed",
  "attempt_id": "6b7cca229ca04dd0bb6e2fad8d1eb7eb",
  "bag": "accepted",
  "failure_type": "",
  "failure_reason": "",
  "started_at": "2026-10-13T09:33:24-03:00",
  "finished_at": "2026-10-13T09:37:47-03:00",
  "attempts": [
    {
      "attempt_id": "a3c8456a2b7443538cd38942153ec4c0",
      "status": "failed",
      "bag": "failed",
      "failure_type": "pipeline_not_found",
      "failure_reason": "Mission completed without finding the pipeline; manually rejected after review.",
      "started_at": "2026-10-12T23:25:18-03:00",
      "finished_at": "2026-10-12T23:29:41-03:00"
    }
  ]
},
```

The runner ends a run immediately when a launched process exits with a
nonzero status or a subsystem publishes `mission/control_failure`. The run is
recorded as `failed` and the runner continues with the remaining runs.

When bag recording is enabled (`record_bags` = `true`), the runner records each run
attempt under `rosbags/pending/` first. When a run completes, the runner validates the MCAP
metadata, storage files, and reader stream, moves the bag to
`rosbags/<manager>_<mission>_<run_id>_<attempt_id>/`, and only then writes
`status: completed` with `bag: accepted` to the `runs.json` file. Failed, interrupted, or
invalid-recording attempts are moved to `rosbags/failed/` (`bag: failed`), so
resume retries them. 

After each run the runner logs a per-manager summary, and `run_batch` logs one
after each campaign. For example:

```Bash
[INFO] [1792333651.235502187] [suave_runner_node]: bt: 2/2 completed; failures={}
[INFO] [1792333651.235856561] [suave_runner_node]: Experiment 1 summary:
[INFO] [1792333651.236125541] [suave_runner_node]: bt: 2/2 completed; failures={}
[INFO] [1792333651.236287649] [suave_runner_node]: All experiment runs completed or aborted.
```

### Batch runner

To run several campaigns sequentially in one go, use the generic `run_batch`
node, configured through a YAML manifest (see
[config/runner/batch_campaigns.yml](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner/batch_campaigns.yml)
for a runnable example). A third-party managing system's own runner config can
be added the same way, check the third-party [PLANTA batch configuration](https://github.com/kas-lab/suave_planta/blob/main/config/runner/batch_campaigns.yml).

With the `suave_cli`:

```bash
suave batch start
suave batch start --config my_batch.yml
```

**Tip:** Check ({doc}`cli`) or run `suave batch --help` for more info on how to use the suave CLI. Dont forget to source the workspace or `env.sh` before using the cli

With the `run_batch` launchfile:

```Bash
ros2 launch suave_runner run_batch_launch.py
```

Or with the run_batch node pointing at any batch manifest:

```Bash
ros2 run suave_runner run_batch \
  --ros-args --params-file config/runner/batch_campaigns.yml
```


The available parameters for a batch manifest (i.e., batch_campaigns.yml) are:

| Parameter | Type | Default | Purpose |
|-----------|------|---------|---------|
| `batch_dir` | string | `''` | New batch output directory; must not already exist. Empty generates `~/suave/results/batches/batch_YYYYMMDD_HHMMSS`. |
| `resume_state_file` | string | `''` | Existing batch `state.json` to resume. Mutually exclusive with `batch_dir`, campaigns are loaded from that file. |
| `fail_fast` | bool | `false` | Stop after a campaign process fails to start or exits nonzero. |
| `dry_run` | bool | `false` | Print campaign commands without executing them or creating batch files. Uses `campaigns`, not the resume manifest. |
| `campaigns` | string array | `['']` | Ordered JSON-encoded objects containing `name`, `package`, and `config_file` (relative to the package share directory). Required for new batches and dry runs; names must be unique. |

**Note:** running every campaign back-to-back can take several hours to a few
days, depending on the machine and `num_runs` per experiment -- plan to leave
it running unattended (e.g. in `screen`/`tmux`) rather than waiting on it.

Running a batch creates a timestamped batch directory under `~/suave/results/batches/`
(e.g. `batch_20260913_104200/`) containing:

- `state.json` -- the batch manifest: the campaign
  list (`name`, `package`, `config_file`), written once when the batch starts.
  Progress is not stored there
- `run_batch.log` -- the overall orchestration log
- `<campaign_name>.log` -- combined stdout/stderr for each campaign (e.g.
  `exp1.log`)
- `campaigns/<campaign_name>/` -- each campaign's own results directory, in
  the same layout `suave_runner` produces for a single campaign


**Checking whether a batch needs to be resumed:** a batch finished cleanly if
`run_batch.log` ends with `All campaigns completed successfully.`. If it
stops early -- Ctrl+C, a crash, a failed campaign -- the log ends instead with
a `Batch interrupted; ...` or `Batch finished with N unfinished campaign(s): ...`
line followed by the exact resume command to use. A campaign is done when the
number of `"status": "completed"` entries in its `campaigns/<name>/runs.json`
equals the number of runs its runner config declares. `suave batch list`
prints the completed runs per campaign, and `diagnose_batch` prints
completed/expected counts.

**Resuming an interrupted batch:** pass the batch's `state.json` as the
`resume_state_file` parameter. Campaigns whose runs have all completed are
skipped, and an unfinished campaign picks up from its `runs.json` instead of
starting over:

With the cli:

```Bash
suave batch resume --latest               # resume the newest batch under the results dir
suave batch resume ~/suave/results/batches/batch_20260926_101500/state.json  # resume a specific batch
```

With the `run_batch` node:

```Bash
ros2 run suave_runner run_batch --ros-args \
  -p resume_state_file:=~/suave/results/batches/<batch_dir>/state.json
```

To run a custom subset or order of campaigns, write your own manifest (or
edit `config/runner/batch_campaigns.yml`) and pass it via `--params-file` before
starting a new batch. `-p fail_fast:=true` stops the batch as soon as a
campaign fails instead of continuing to the next one, and `-p dry_run:=true`
prints the `ros2 run` command for each campaign without executing anything.

### Diagnosing batch failures

`diagnose_batch` reads a batch's or campaign's result files and reports which
runs failed, why, and where the evidence is. It only reads files, so it works
on a finished batch, a running one, or a copy on another machine:

```bash
ros2 run suave_runner diagnose_batch ~/suave/results/batches/<batch_dir>/state.json
ros2 run suave_runner diagnose_batch ~/suave/results/batches/<batch_dir>
ros2 run suave_runner diagnose_batch ~/suave/results/<campaign_folder>
ros2 run suave_runner diagnose_batch <path> --json
```

With the `suave` CLI: `suave batch diagnose --latest` or
`suave batch diagnose <path> [--json]`.

For each campaign it prints the completed and expected run counts, one line
per adaptation manager (for example
`bt: 18/20 completed; failures={'mission_done_timeout': 2}`), and, for each
run that did not complete, its status, failure type, reason, log folder, and
the first log lines that point at a cause (`suspected <type>: <log>:<line>`).
It reads each campaign's `runs.json` and, for a batch, the manifest.

| Failure type | Meaning |
|---|---|
| `mission_done_timeout` | No `mission_metrics/done` within `run_duration` |
| `process_died_before_completion` | A launched process exited with a nonzero status |
| `mission_control_failure` | A subsystem published `mission/control_failure`. This includes the `lifecycle_watchdog` node (launched by `suave.launch.py`), which reports a lifecycle node that did not become active within `activation_timeout` (10 s) of `mode_manager`'s activate request. |
| `readiness_timeout` | ArduPilot or the simulation was not ready within `ardupilot_ready_timeout` / `simulation_ready_timeout` |
| `readiness_missing_service` | Services in the experiment's `required_services` were missing after `readiness_timeout` |
| `startup_failure` | Launching the run's processes raised an error |
| `interrupted` | The run was stopped by Ctrl+C or termination |
| `unknown` | No recognizable cause was recorded |

**Systemic warnings:** a line `SYSTEMIC: <manager>: repeated <failure_type>`
means that manager failed with the same type at least 3 times, or at least
twice and in more than half of its attempted runs. That usually indicates there is
a bug in either SUAVE (the managed system) or in the managing system, an incorrect experiment configuration, 
or something wrong in the environment (e.g., in the docker container or your host machine) where the experiments are being executed (e.g., a missing package, wrong dependency version etc) rather than a random failure. So, if you keep getting systemic warnings, we suggest you inspect one failed run's logs and try to fix the issue before rerunning.

### Resuming a crashed campaign

If the runner is interrupted mid-campaign (crash, Ctrl+C, power loss), it can
resume from where it left off without re-running completed runs. To resume, pass 
the path of the interrupted result folder via `resume_result_path`:

With the cli:

```Bash
suave batch resume --latest               # resume the newest batch under the results dir
suave batch resume ~/suave/results/batches/batch_20260926_101500/state.json  # resume a specific batch
```

With the `suave_runner` node:

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
(`mission_config_run*.yaml`) are reused from the existing folder if present,
otherwise they are regenerated using the same random seed. 

**How it resuming works under the hood:**

Before starting new runs, a
resumed runner settles every run left `running` (or `interrupted` with a
`pending` bag): it confirms the attempt's launch session has no live processes
(recovery only checks and never kills processes), then decides from where the
bag is on disk. A bag only under `rosbags/` is validated and the run becomes
`completed`, or `failed` with the bag quarantined if it is invalid. A bag 
under `rosbags/pending/` is moved to `rosbags/failed/` and the run becomes
`interrupted`, with no bag the run becomes `interrupted`, a bag in both places
stops recovery with an error. The accepted bag of every completed run is
re-validated too, and a missing one stops recovery instead of rerunning the
trial. 

Before every attempt, if a live `gz sim` process exists or UDP port 9002 (the BlueROV2
`ArduPilotPlugin` `fdm_port_in`) is bound, the runner refuses to start, raising
`StaleSimulationError` and stopping the campaign without recording an attempt.
If this happens, stop the leftover `gz sim` server and/or ardupilot and resume the campaign. 

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
