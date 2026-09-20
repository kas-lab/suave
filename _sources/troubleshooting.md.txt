# Troubleshooting

List of known problems and their solutions. If you find new problems and find a solution for it, please consider contributing to this section.

## MAVROS and ArduSub integration problems:

A common problem that occurs over time is that some packages are upgraded in Ubuntu 22.04 and the connection between MAVROS and ArduSub stops working. I don't know how to fix this issue long term, but a workaround is to update MAVROS and/or ArduSub and check if it works again.

Before updating MAVROS and ArduSub upgrade your Ubuntu:

```Bash
sudo apt update && sudo apt upgrade
```

**Update ArduSub:**

Due to ArduSub usage of submodules, it is simpler to remove the whole ArduPilot repository and build it from scratch again.

```Bash
rm -rf ~/ardupilot
```

SUAVE is tested with [ArduSub commit `571e8c7`](https://github.com/ArduPilot/ardupilot/tree/571e8c7bd3793fce1bc5184a2f6586feb8a616e5). To test a different version, select a branch or commit from the [ArduPilot repository](https://github.com/ArduPilot/ardupilot), follow the [ArduSub installation instructions](installation.md#install-ardusub), and rebuild the workspace. ArduPilot, MAVROS, and the Gazebo plugin must remain mutually compatible.

**Update MAVROS:**
To update MAVROS, change its version in the [suave.repos](https://github.com/kas-lab/suave/blob/main/suave.repos) file with the newest version of [mavros](https://github.com/mavlink/mavros), or simply change the version to `ros2`. Then pull the repo:

```Bash
cd ~/suave_ws/
vcs pull src
```

Alternatively, instead of updating the [suave.repos](https://github.com/kas-lab/suave/blob/main/suave.repos) file, you can just update mavros manually:

```Bash
cd ~/suave_ws/src/mavros
git checkout ros2
git pull
```

Don't forget to rebuild the suave workspace:
```Bash
cd ~/suave_ws/
colcon build --symlink-install
```
