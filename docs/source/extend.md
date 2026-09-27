# Extending SUAVE and connecting managing subsystems

## Connecting managing subsystems

SUAVE exposes several ROS 2 interfaces that managing subsystems can use at different abstraction levels. A managing subsystem does not need to use all of them. It should use the subset that matches its control strategy.

1. The `/diagnostics` topic publishes monitoring information using the [diagnostic_msgs/DiagnosticArray](https://docs.ros.org/en/humble/p/diagnostic_msgs/msg/DiagnosticArray.html) message type. Common QA keys include `water_visibility`, `water_current`, `battery_level`, `coverage_area`, `operational_thrusters`, and `c_thruster_<N>`.
2. The optional `/task/request` and `/task/cancel` services provide a high-level task interface through `task_bridge`. Both services use the [suave_msgs/Task](https://github.com/kas-lab/suave/blob/main/suave_msgs/srv/Task.srv) service type. They are useful for clients or managers that want to request abstract mission tasks instead of controlling individual functions directly.
3. Three [system_modes](https://github.com/micro-ROS/system_modes) services provide a function-level reconfiguration interface. These services use the [system_modes_msgs/ChangeMode](https://github.com/micro-ROS/system_modes/blob/master/system_modes_msgs/srv/ChangeMode.srv) service type:
    1. Service `/f_maintain_motion/change_mode` to change the Maintain Motion node modes
    2. Service `/f_generate_search_path/change_mode` to change the Generate Search Path node modes
    3. Service `/f_follow_pipeline/change_mode` to change the Follow Pipeline node modes

    Managers such as the Behavior Tree manager subscribe to `/diagnostics` and call these services directly.
4. A managing subsystem may also bypass `task_bridge` and `system_modes` entirely and control the managed ROS 2 nodes directly through standard ROS 2 lifecycle transition services and parameter services. This lower-level integration is more tightly coupled to the managed nodes, but it can be appropriate for managers that need direct lifecycle or parameter control.

Thus, to connect a managing subsystem to SUAVE, choose the integration level that matches the manager. A task-level manager uses `/task/request` and `/task/cancel`. A function-level manager typically uses `/diagnostics` plus the `system_modes` services. A low-level manager may use `/diagnostics` plus ROS 2 lifecycle and parameter APIs directly.

There are two ways to wire a new managing subsystem into a running SUAVE stack depending on where the manager lives.

### Option A — External package (recommended for standalone managing systems)

Add `suave_base` as an `exec_depend` in your package's `package.xml`:

```xml
<exec_depend>suave_base</exec_depend>
```

In your launch file, include `suave_base.launch.py` to start the managed system and metrics, then add your manager nodes:

```python
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

suave_base = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('suave_base'),
            'launch', 'suave_base.launch.py')),
    launch_arguments={
        'adaptation_manager': 'my_manager',
        'result_path': result_path,
    }.items())

# ... your manager nodes below
```

`suave_base.launch.py` starts the managed system (with `task_bridge` disabled — your manager owns task routing) and the mission metrics node. It does **not** start a mission node, add that separately if your manager relies on it.

### Option B — Built-in manager (contributed upstream to the suave repo)

Create a manager-only launch file (see [suave_metacontrol.launch.py](https://github.com/kas-lab/suave/blob/main/suave_managing/suave_metacontrol/launch/suave_metacontrol.launch.py) for an example). The manager launch must not start the managed system, mission node, or metrics — `suave_bringup` owns that composition.

Declare the manager package as an `exec_depend` of `suave_bringup`, then add it to [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_bringup/launch/mission.launch.py) behind an `adaptation_manager` condition:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('[new_managing_subsystem]'),
            'launch', '[new_managing_subsystem].launch.py')),
    condition=LaunchConfigurationEquals(
        'adaptation_manager', '[new_managing_subsystem]'))
```

## Extend SUAVE

To extend SUAVE with new functionalities, add lifecycle nodes that implement the new functionalities (check [spiral_search_lc.py](https://github.com/kas-lab/suave/blob/main/suave/suave/spiral_search_lc.py) for an example), and add their modes to the [system_modes](https://github.com/micro-ROS/system_modes) configuration file [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml). If you create a new configuration file, replace the `suave_modes.yaml` path used by the system-modes launch file.

### Optional ROS 2 action execution

Managed lifecycle nodes may additionally expose a ROS 2 action server. Existing
nodes use the Boolean `use_action_server` parameter, defaulting to `false`, to
select whether behavior starts on lifecycle activation or after an accepted
goal. Create action servers during lifecycle configuration so they remain
discoverable, stop active callbacks before cleanup, and support cancellation
inside long-running waits. New action definitions belong in `suave_msgs/action/`.

ROS 2 actions are an optional execution mechanism and do not replace the
diagnostics, task-level, function-level, or low-level lifecycle and parameter
interfaces selected by the managing subsystem.
