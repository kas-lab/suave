# Extending SUAVE and connecting managing subsystems

## Connecting managing subsystems

SUAVE is designed to allow for different managing subsystems to be used, as long as they adhere to the correct ROS 2 interfaces.
SUAVE's ROS2 interfaces are:

1. The `/diagnostics` topic, which is where monitoring information is published. This topic uses the [diagnostic_msgs/DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html) message type
2. The `/task/request` and `/task/cancel` services, which are used to request and cancel tasks, respectively. Both services use the [suave_msgs/Task](https://github.com/kas-lab/suave/blob/main/suave_msgs/srv/Task.srv) service type
3. Three [system_modes](https://github.com/micro-ROS/system_modes) services to change SUAVE's LifeCycle nodes mode. These services use the [system_modes_msgs/ChangeMode](https://github.com/micro-ROS/system_modes/blob/master/system_modes_msgs/srv/ChangeMode.srv) service type:
    1. Service `/f_maintain_motion/change_mode` to change the Maintain Motion node modes
    2. Service `/f_generate_search_path/change_mode` to change the Generate Search Path node modes
    3. Service `/f_follow_pipeline/change_mode` to change the Follow Pipeline node modes

Thus, to connect a different managing subsystem to SUAVE, it must subscribe to `/diagnostics` to get monitoring information, send adaptation goals (task) requests via `/task/request` and `/task/cancel`, and send reconfiguration requests via `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, or `/f_follow_pipeline/change_mode`.


In order to use the new managing subsystem with the launchfile [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_missions/launch/mission.launch.py) as explained in the run suave section, a new launchfile must be created for the new managing subsystem (check [suave_metacontrol.launch.py](https://github.com/kas-lab/suave/blob/main/suave_metacontrol/launch/suave_metacontrol.launch.py) for an example), and the new launch file must be included in the [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_missions/launch/mission.launch.py) file.


The new launch file must include SUAVE's base launch:
```python
suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

suave_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(suave_launch_path),
    launch_arguments={
        'task_bridge': 'False'}.items()
)
```

To include it in [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_missions/launch/mission.launch.py), add the following code replacing \[new_managing_subsystem\] with the proper name:

```python
[new_managing_subsystem]_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        '[new_managing_subsystem].launch.py')

[new_managing_subsystem]_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([new_managing_subsystem]_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager',
                    '[new_managing_subsystem]'))
```

## Extend SUAVE

To extend SUAVE with new functionalities, it is only required to add new LifeCycle nodes that implement the new functionalities (check [spiral_search_lc.py](https://github.com/kas-lab/suave/blob/main/suave/suave/spiral_search_lc.py) for an example), and add its different modes to the [system_modes](https://github.com/micro-ROS/system_modes) configuration file [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml). Note, that if you create a new configuration file, you should replace the [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml) path with the new file path.
