# suave_bt

`suave_bt` is the BehaviorTree.CPP managing subsystem for SUAVE. It selects
system modes from diagnostics and coordinates pipeline search, inspection,
thruster recovery, and battery recharge. The standard and extended trees are
defined in `bts/suave.xml` and `bts/suave_extended.xml`.

## Running

Run the standard tree with the legacy lifecycle behavior:

```bash
ros2 launch suave_bt suave_bt.launch.py
```

Run it using ROS 2 actions:

```bash
ros2 launch suave_bt suave_bt.launch.py use_action_server:=true
```

The extended mission supports the same argument:

```bash
ros2 launch suave_bt suave_bt_extended.launch.py use_action_server:=true
```

## `use_action_server`

`use_action_server` is a Boolean launch argument and ROS parameter that
defaults to `false`.

- `false`: activating a managed lifecycle node starts its behavior. The BT
  observes legacy completion topics.
- `true`: lifecycle activation only prepares the behavior. The BT sends goals
  to `spiral_search`, `follow_pipeline`, `recover_thrusters`, and
  `recharge_battery`, and uses their action results to determine BT status.

The launch file passes this value to both the managed lifecycle nodes and the
BT mission node. The mission node stores it on the BT blackboard so all action
nodes use the same mode. Do not configure the two sides independently.

For a `suave_runner` campaign, include the argument in the experiment command
in `suave_runner/config/runner_config.yml`:

```json
{
  "experiment_launch": "ros2 launch suave_bt suave_bt.launch.py use_action_server:=true",
  "num_runs": 10,
  "adaptation_manager": "bt",
  "mission_name": "suave"
}
```
