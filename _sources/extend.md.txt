# Extending SUAVE and connecting managing subsystems

## Connecting managing subsystems

SUAVE is designed to allow for different managing subsystems to be used, as long as they adhere to the correct ROS 2 interfaces.
SUAVE's ROS 2 interfaces are:

1. The `/diagnostics` topic, which is where monitoring information is published. This topic uses the [diagnostic_msgs/DiagnosticArray](https://docs.ros.org/en/humble/p/diagnostic_msgs/msg/DiagnosticArray.html) message type
2. The `/task/request` and `/task/cancel` services, which are used to request and cancel tasks, respectively. Both services use the [suave_msgs/Task](https://github.com/kas-lab/suave/blob/main/suave_msgs/srv/Task.srv) service type
3. Three [system_modes](https://github.com/micro-ROS/system_modes) services to change SUAVE's lifecycle-node modes. These services use the [system_modes_msgs/ChangeMode](https://github.com/micro-ROS/system_modes/blob/master/system_modes_msgs/srv/ChangeMode.srv) service type:
    1. Service `/f_maintain_motion/change_mode` to change the Maintain Motion node modes
    2. Service `/f_generate_search_path/change_mode` to change the Generate Search Path node modes
    3. Service `/f_follow_pipeline/change_mode` to change the Follow Pipeline node modes

Thus, to connect a different managing subsystem to SUAVE, it must subscribe to `/diagnostics` to get monitoring information, send adaptation goals (task) requests via `/task/request` and `/task/cancel`, and send reconfiguration requests via `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, or `/f_follow_pipeline/change_mode`.

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
standard diagnostics, task, and mode-change interfaces required from managing
subsystems.
