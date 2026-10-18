---
name: suave-python-node-diagnostics
description: Use when adding or modifying Python ROS 2 nodes in the SUAVE managed-system packages, especially nodes that publish quality-attribute diagnostics on /diagnostics.
---

# SUAVE Python Node + Diagnostics Skill

Use this skill for changes under `src/suave/suave/suave/` and closely related SUAVE Python node work.

## Required context

Before editing:

1. Read `src/suave/AGENTS.md`.
2. Check package status and preserve unrelated changes:
   ```bash
   git -C src/suave status --short
   ```
3. Inspect nearby nodes for current package style before inventing structure.

Useful examples:

- `src/suave/suave/suave/pipeline_detection.py`
- `src/suave/suave/suave/pipeline_detection_wv.py`
- `src/suave/suave/suave/spiral_search_lc.py`
- `src/suave/suave_monitor/suave_monitor/water_visibility_observer.py`

## File and package conventions

- Existing SUAVE Python nodes are plain `rclpy.node.Node` classes.
- Public modules, classes, constructors, methods, and `main()` functions need docstrings.
- Use the SUAVE Apache-2.0 header for new Python files:
  ```python
  # Copyright 2026 KAS Lab
  #
  # Licensed under the Apache License, Version 2.0 (the "License");
  # you may not use this file except in compliance with the License.
  # You may obtain a copy of the License at
  #
  #     http://www.apache.org/licenses/LICENSE-2.0
  #
  # Unless required by applicable law or agreed to in writing, software
  # distributed under the License is distributed on an "AS IS" BASIS,
  # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  # See the License for the specific language governing permissions and
  # limitations under the License.
  ```
- Register executable nodes in `src/suave/suave/setup.py` under `entry_points['console_scripts']`.
- Declare ROS dependencies in `src/suave/suave/package.xml` when new message packages are introduced.

## Parameters

- Declare parameters in `__init__`.
- Prefer `rcl_interfaces.msg.ParameterDescriptor` descriptions for new public parameters.
- If a node is launched from `suave.launch.py` and should be configurable by mission files, pass `parameters=[mission_config]` from the launch file and add defaults under the node name in `src/suave/suave_missions/config/mission_config.yaml`.
- Use node names consistently between launch and YAML, e.g. `/water_current:` in YAML for `name='water_current'`.

## Diagnostics / QA status pattern

SUAVE quality-attribute outputs usually publish to `/diagnostics` as `diagnostic_msgs/msg/DiagnosticArray`.

Use the pattern from `water_visibility_observer.py`:

```python
key_value = KeyValue()
key_value.key = '<qa_name>'
key_value.value = str(value)

status_msg = DiagnosticStatus()
status_msg.level = DiagnosticStatus.OK
status_msg.name = '<node_name>: <human readable measurement>'
status_msg.message = 'QA status'
status_msg.values.append(key_value)

diag_msg = DiagnosticArray()
diag_msg.header.stamp = self.get_clock().now().to_msg()
diag_msg.status.append(status_msg)

diagnostics_publisher.publish(diag_msg)
```

For vector-valued QAs represented as a string, use a space-separated array-like value when requested by existing consumers:

```python
key_value.value = f'[{x} {y} {z}]'
```

## MAVROS state / experiment-time convention

When a time-varying disturbance or QA must be independent of startup delays, follow the water-visibility convention:

- Subscribe to `mavros/state` (`mavros_msgs/msg/State`).
- Keep experiment time at zero until `msg.mode == 'GUIDED'`.
- Store the first GUIDED time.
- Destroy the state subscription after GUIDED is observed if no longer needed.

## Main function pattern

Typical node main:

```python
def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Validation

Run focused checks inside the applicable SUAVE container when possible:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m py_compile <changed.py> && python3 -m flake8 <changed.py> && python3 -m pydocstyle <changed.py>'
```

For package-level verification:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select suave --event-handlers console_direct+'
```

If using the integration workspace container, follow the root `AGENTS.md` Docker command pattern instead.
