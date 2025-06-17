# Troubleshooting

List of known problems and their solutions. If you find new problems and find a solution for it, please consider contributing to this section.

## MAVROS and ArduSub integration problems:

A common problem that occurs over time is that some packages are upgraded in Ubuntu 22.04 and the connection between MAVROS and ArduSub stops working. I don't know how to fix this issue long term, but a workaround is to update MAVROS and/or ArduSub and check if it works again.

Before updating MAVROS and ArduSub upgrade your Ubuntu:

```Bash
sudo apt update && sudo apt upgrade
```

**Update ArduSub:**

Due to ArduSub usage of submodules, it is simpler to just remove the whole ardupilo repo and build it from scratch again.

```Bash
rm -rf ~/ardupilot
```

To find the latest version of ArduSub go to the [ardupilot repo](https://github.com/ArduPilot/ardupilot) and look for the newest branch of ArduSub. At the time of this writing, the latest branch is [Sub-4.1](https://github.com/ArduPilot/ardupilot/commits/Sub-4.1) at commit [e9f46b9](https://github.com/ArduPilot/ardupilot/tree/e9f46b91cda16cf7a48eb9861fea13e452c5c08c). After you know the latest branch or commit you want to get, follow the [install ardusub](https://github.com/kas-lab/suave#install-ardusub) instructions replacing the commit in `git checkout e9f46b9` with the commit/branch you selected.

**Update MAVROS:**
To update MAVROS, you can either change its version in the [suave.rosinstall](https://github.com/kas-lab/suave/blob/8f0e47571fc6c7524139fcb7ef20d9157d73a3e6/suave.rosinstall#L9) file with the newest version of [mavros](https://github.com/mavlink/mavros), or simply change the version to ros2. Then you need to pull the repo:

```Bash
cd ~/suave_ws/
vcs pull src
```

Alternatively, instead of updating the suave.rosinstall file, you can just update mavros manually:

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
