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
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py\", \
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
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py\", \
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
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py use_action_server:=true\", \
      \"num_runs\": 2, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

You can also use a launch file with a [config file](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner_config.yml) to make it easier to run the experiments:

```Bash
ros2 launch suave_runner suave_runner_launch.py
```

To run SUAVE with different managing subsystems, replace the `experiment_launch` with the proper launch file.

### Runner config reference

`suave_runner/config/runner_config.yml` controls all experiment parameters. The key fields are:

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
      "experiment_launch": "ros2 launch suave_bt suave_bt.launch.py",
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

### Resuming a crashed campaign

If the runner is interrupted mid-campaign (crash, Ctrl+C, power loss), it can
resume from where it left off without re-running completed runs.

After each **successful** run the runner writes a marker file
`run_<exp_idx>_<run_idx>.done` inside the result folder. Runs that timed out
(no result received within `run_duration`) are **not** marked and will be
retried on resume.

To resume, pass the path of the interrupted result folder via
`resume_result_path`:

```Bash
ros2 run suave_runner suave_runner \
  --ros-args \
  -p resume_result_path:=~/suave/results/2026_06_19_10-30-00 \
  -p experiments:='[
    "{\"experiment_launch\": \"ros2 launch suave_bt suave_bt.launch.py\", \
      \"num_runs\": 10, \
      \"adaptation_manager\": \"bt\", \
      \"mission_name\": \"suave\"}"
  ]'
```

The `experiments` parameter must match the original campaign exactly so that
run indices line up with the marker files. Per-run mission config files
(`mission_config_run*.yaml`) are reused from the existing folder if present;
otherwise they are regenerated using the same random seed.

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
ros2 launch suave_missions mission.launch.py
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
    (default: empty; the selected manager uses its metrics default)

'mission_type':
    BT mission label written to metrics
    (default: 'time_constrained_mission')

'use_action_server':
    For the BT manager, start managed behaviors through ROS 2 actions
    available values: true/false
    (default: false)
```

The arguments can be defined by adding the above arguments with the notation `<name>:=<value>` to the end of the command line.

An example of running a mission with metacontrol saving to a file called 'measurement_1':

```Bash
ros2 launch suave_missions mission.launch.py adaptation_manager:=metacontrol result_filename:=measurement_1
```

An example of running the BT manager in action-server mode:

```Bash
ros2 launch suave_missions mission.launch.py \
  adaptation_manager:=bt use_action_server:=true
```
