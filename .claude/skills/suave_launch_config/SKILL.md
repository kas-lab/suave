---
name: suave-launch-config
description: Use when modifying SUAVE launch files, optional nodes, mission configuration YAML, or launch arguments in the SUAVE workspace.
---

# SUAVE Launch + Mission Config Skill

Use this skill for changes to SUAVE launch files and mission configuration, especially:

- `src/suave/suave/launch/*.launch.py`
- `src/suave/suave_bringup/launch/*.launch.py`
- `src/suave/suave_missions/config/*.yaml`
- launch-time optional managed-system nodes

## Required context

Before editing:

1. Read `src/suave/AGENTS.md`.
2. Check status and preserve unrelated changes:
   ```bash
   git -C src/suave status --short
   ```
3. Inspect the relevant launch file and nearby patterns before editing.

## Launch-file ownership conventions

- `simulation.launch.py` owns simulator-specific processes and Gazebo bridges, including `ros_gz_bridge` nodes and model spawning.
- `suave.launch.py` owns managed-system runtime nodes and supporting monitors.
- `suave_bringup/launch/mission.launch.py` composes mission-level bringup and adaptation-manager selection.
- Prefer keeping Gazebo-specific bridges in simulation launch files and ROS managed-system nodes in `suave.launch.py` unless the user requests otherwise.

## Launch argument conventions

Use `DeclareLaunchArgument` plus `LaunchConfiguration`:

```python
my_arg = LaunchConfiguration('my_arg')
my_arg_declare = DeclareLaunchArgument(
    'my_arg',
    default_value='false',
    description='...'
)
```

Add the `DeclareLaunchArgument` object to the returned `LaunchDescription` before nodes that conceptually use it.

For optional nodes, use:

```python
from launch.conditions import IfCondition

Node(
    package='...',
    executable='...',
    condition=IfCondition(my_arg),
)
```

For existing True/False string arguments in SUAVE that use exact matching, follow nearby `LaunchConfigurationEquals` patterns.

## Mission config YAML conventions

The default mission file is usually:

```text
src/suave/suave_missions/config/mission_config.yaml
```

When a node should read mission defaults:

1. Pass the mission config into the node:
   ```python
   parameters=[mission_config]
   ```
2. Ensure the launch node name matches the YAML key:
   ```python
   Node(name='water_current', ...)
   ```
   ```yaml
   /water_current:
     ros__parameters:
       parameter_name: value
   ```
3. Keep inline YAML comments concise and consistent with nearby entries.

## Common node patterns

Monitor/support nodes often use:

```python
Node(
    package='suave_monitor',
    executable='water_visibility_observer',
    name='water_visibility_observer_node',
    parameters=[mission_config],
    output=print_output,
)
```

Managed-system action-capable nodes pass launch configurations directly as parameters:

```python
Node(
    package='suave',
    executable='spiral_search',
    parameters=[{'use_action_server': use_action_server}],
    output=print_output,
)
```

Optional managed-system support nodes should usually live in `suave.launch.py`, not `simulation.launch.py`, while their Gazebo bridge remains in `simulation.launch.py`.

## Formatting and imports

- Keep imports grouped and flake8-clean.
- `launch_ros.actions.Node` is separated from `launch` imports by a blank line in existing files.
- Avoid long launch argument strings beyond the package flake8 limit.

## Validation

For Python launch files:

```bash
python3 -m py_compile <launch_file.py>
python3 -m flake8 <launch_file.py>
python3 -m pydocstyle <launch_file.py>
```

For YAML:

```bash
python3 - <<'PY'
import yaml
with open('<config.yaml>') as f:
    yaml.safe_load(f)
PY
```

Inside the SUAVE container, run package tests when practical:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select suave suave_missions --event-handlers console_direct+'
```

After changing package data, launch files, setup metadata, or installed config files, rebuild with `colcon build --symlink-install` before expecting installed resources to change.
