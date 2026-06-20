# SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/kas-lab/suave)](https://github.com/kas-lab/suave/releases)
[![DOI](https://zenodo.org/badge/530141277.svg)](https://zenodo.org/badge/latestdoi/530141277)
[![Main](https://github.com/kas-lab/suave/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/kas-lab/suave/actions/workflows/main.yml)
[![Docker Images](https://github.com/kas-lab/suave/actions/workflows/container.yml/badge.svg)](https://github.com/kas-lab/suave/actions/workflows/container.yml)

This repository contains SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar). SUAVE focuses on the scenario of pipeline inspection
for a single autonomous underwater vehicle (AUV). The AUV’s mission is to first search for a pipeline on the seabed, then follow and inspect the pipeline.

It clearly separates the system into two subsystems: the managed subsystem and the managing subsystem. The managed subsystem implements the functionalities required by the AUV to perform the pipeline inspection mission, and the managing subsystem implements the adaptation logic.  This ensures that this exemplar can be reused with several different managing subsystems, providing that they satisfy the necessary [requirements](#requirements-for-a-managing-subsystem). The usability of the exemplar is showcased with [MROS2](https://github.com/meta-control/mc_mros_reasoner), the implementation of the self-adaptation framework [Metacontrol](https://research.tudelft.nl/en/publications/model-based-self-awareness-patterns-for-autonomy).

This repository is organized as following:
- The package [suave](https://github.com/kas-lab/suave/tree/main/suave) contains the managed subsystem functionalities
- The package [suave_monitor](https://github.com/kas-lab/suave/tree/main/suave_monitor) contains the monitor nodes
- The package [suave_missions](https://github.com/kas-lab/suave/tree/main/suave_missions) contains the AUV's missions
- The package [suave_metrics](https://github.com/kas-lab/suave/tree/main/suave_metrics) contains a node used for collecting mission metrics
- The package [suave_metacontrol](https://github.com/kas-lab/suave/tree/main/suave_managing/suave_metacontrol) contains the metacontrol implementation of the managing subsystem
- The package [suave_random](https://github.com/kas-lab/suave/tree/main/suave_managing/suave_random) contains the implementation of a random managing subsystems
- The package [suave_bt](https://github.com/kas-lab/suave/tree/main/suave_managing/suave_bt) contains the behavior tree implementation of the managing subsystem
- The package [suave_msgs](https://github.com/kas-lab/suave/tree/main/suave_msgs) contains suave's specific ros msgs
- The folder [docker](https://github.com/kas-lab/suave/tree/main/docker) contains the dockerfiles and scripts used to package this repository
- The folder [runner](https://github.com/kas-lab/suave/tree/main/runner) contains the bash scripts to run the exemplar

A video of the SUAVE running (click in the image to open the video):

<p align="center">
  <a href="https://www.youtube.com/watch?v=X8yUZjM5bfk"/>
  <img src="docs/images/image.png" width="500">
</p>


An overview of the system:

<p align="center">
  <img src="https://user-images.githubusercontent.com/20564040/227582678-e1494c89-1b44-4bac-aef6-bdd77127dfa6.png" width="500">
</p>

The exemplar can either be used with [Docker](#use-suave-with-docker) or [installed locally](#install-suave-locally). The exemplar can be executed following this [instructions](#run-suave).

A paper describing this exemplar was presented at SEAMS 2023 artifact track, you can find it [here](https://ieeexplore.ieee.org/abstract/document/10173938). And an open access pre-print can be found [here](https://arxiv.org/abs/2303.09220).

## Navigate the README
- [Use SUAVE with Docker](#use-suave-with-docker)
  - [Use the headless image](#use-the-headless-image)
- [Install SUAVE locally](#install-suave-locally)
- [Run SUAVE](#run-suave)
- [Extending SUAVE and connecting managing subsystems](#extending-suave-and-connecting-managing-subsystems)
- [Related repository](#related-repository)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)

## Use SUAVE with Docker

You can pull and run the exemplar as a Docker container using the following command. Keep in mind you need to have [Docker](https://docs.docker.com/get-docker/) installed on your computer and running.

In a terminal on your computer run:
```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main
```

Optionally you can add the parameter `-v <absolute_path_host_compute>:/home/kasm-user/suave/results` to save the results into your computer, replace `<absolute_path_host_compute>` with the absolute path of where you want the data to be saved in your computer, e.g:

```Bash
docker run -it --shm-size=512m -v $HOME/suave_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:main
```

**SEAMS2023:** To use the docker image used in the SEAMS2023 paper, replace `ghcr.io/kas-lab/suave:main` with `ghcr.io/kas-lab/suave:seams2023`.

Once the container is up and running, you can interface with it through your web browser. The container will be hosted locally at the port specified, in this case 6901. So in your browser, go to
`http://localhost:6901`.

A dialog will request a username and password, these are shown below, with the password being specifiable in the run command.

 - **User** : `kasm_user`
 - **Password**: `password`

Now you can proceed to [run the exemplar](#run-suave).

### Use the headless image

The headless image (`ghcr.io/kas-lab/suave-headless:main`) is designed for automated experiment campaigns on servers or CI machines where no display is available. It uses a virtual framebuffer (Xvfb) so Gazebo can still run off-screen.

Pull and run an experiment campaign:

```Bash
docker run -it --shm-size=512m \
  ghcr.io/kas-lab/suave-headless:main \
  ./src/suave/runner/headless_runner.sh false metacontrol time 2
```

The runner arguments are the same as `runner.sh`:
1. `true` or `false` — whether to show a GUI (use `false` for headless)
2. `metacontrol`, `random`, `none`, or `bt` — adaptation manager
3. `time` or `distance` — mission type
4. Number of runs (integer)

To save results to your host machine, mount a volume to `/home/ubuntu-user/suave/results`:

```Bash
docker run -it --shm-size=512m \
  -v $HOME/suave_results:/home/ubuntu-user/suave/results \
  ghcr.io/kas-lab/suave-headless:main \
  ./src/suave/runner/headless_runner.sh false metacontrol time 2
```

**NVIDIA GPU support:** If your machine has an NVIDIA GPU, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and use:

```Bash
docker run -it --rm --gpus all --runtime=nvidia --shm-size=512m \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v $HOME/suave_results:/home/ubuntu-user/suave/results \
  ghcr.io/kas-lab/suave-headless:main \
  ./src/suave/runner/headless_runner.sh false metacontrol time 2
```

### Build Docker images locally
To build the docker images locally, run:

```Bash
./build_docker_images.sh
```

The script builds the browser image as `suave:dev` and the headless runner image as `suave-headless:dev` from the local checkout.

## Install SUAVE locally
To install the exemplar locally, you have to [install Gazebo Harmonic](#install-gazebo-harmonic), [install ROS2 Humble](#install-ros2-humble), [install ArduSub](#install-ardusub), [install the ArduSub plugin](#install-ardusub_plugin), and finally [install the SUAVE workspace](#install-suave-workspace).

#### Install Gazebo Harmonic

Follow the [official instructions](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu) for installing Gazebo Harmonic.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble.

#### Install ROS_GZ

```Bash
apt install ros-humble-ros-gzharmonic
```

#### Install ArduSub
ArduSub is a subproject within ArduPilot for piloting underwater vehicles.

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions. This repo was tested with [ArduSub commit `571e8c7`](https://github.com/ArduPilot/ardupilot/tree/571e8c7bd3793fce1bc5184a2f6586feb8a616e5) (ArduSub 4.7.0-beta4) and [mavros 2.14.0](https://github.com/mavlink/mavros/tree/2.14.0).

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 571e8c7bd3793fce1bc5184a2f6586feb8a616e5
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

To test if the installation worked, run:

```Bash
sim_vehicle.py -v ArduSub -L RATBeach --console --map
```
ArduPilot SITL should open and a console plus a map should appear.

#### Install the ArduSub plugin

Install the dependencies:

```Bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

```Bash
export GZ_VERSION=harmonic
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
rosdep resolve gz-harmonic
# Navigate to your ROS workspace before the next command.
rosdep install --from-paths src --ignore-src -y
```

Clone and build the repository:

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
git checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Add the required paths:

Assuming that you have cloned the repository in `$HOME/ardupilot_gazebo`, run:
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Now that new environmental variables have been added to your terminal, you need to reload it with
```bash
source ~/.bashrc
```
More info about the plugin can be found in the corresponding [repository](https://github.com/ArduPilot/ardupilot_gazebo/).

### Install the SUAVE workspace

Create the workspace and download the required repositories:
```Bash
mkdir -p ~/suave_ws/src/
cd ~/suave_ws/
```

If you want to get the most updated version of the repo:

```Bash
wget https://raw.githubusercontent.com/kas-lab/suave/main/suave.repos
vcs import src < suave.repos --recursive
```
For a reproducible release installation, download `suave.repos` from the
matching Git tag instead of `main`.

**SEAMS 2023 artifact:** The exact source submitted with the published paper
is preserved at commit
[`9e64688`](https://github.com/kas-lab/suave/tree/9e6468896ce766376557ca9522d84f92b70129f1).
Because that artifact uses a historical dependency set, the recommended way
to reproduce it is the archived `ghcr.io/kas-lab/suave:seams2023` container
image. Use the `main` or release-tag instructions above for current
development.

Before building the `ros_gz` package (one of the dependencies), you need to export the gazebo version:

```
export GZ_VERSION="harmonic"
```
You can also add this to your `~/.bashrc` to make this process easier.

Install Python dependencies:
```Bash
pip install -r src/suave/requirements.txt
```

Install the ROS dependencies:
```Bash
source /opt/ros/humble/setup.bash
cd ~/suave_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Build the project:
```Bash
cd ~/suave_ws/
colcon build --symlink-install
```

If you have memory problems while building the package, run the following command instead, it is slower but uses less memory:

```Bash
colcon build --symlink-install --executor sequential --parallel-workers 1
```

Install a MAVROS dependency:
```Bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

Now you can proceed to [run the exemplar](#run-suave).

## Run SUAVE

### Runner
#### Trying it out!
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

#### Full Runner

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

```Bash
cd ~/suave_ws/
source install/setup.bash
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

Or using a launch file with a [config file](https://github.com/kas-lab/suave/blob/main/suave_runner/config/runner_config.yml):

```Bash
ros2 launch suave_runner suave_runner_launch.py
```

To run the BT manager through ROS 2 actions, add
`use_action_server:=true` to the experiment launch command, for example:

```text
ros2 launch suave_bt suave_bt.launch.py use_action_server:=true
```

### Without the runner

**Configuring SUAVE:**
SUAVE has a number of parameters that may be of interest when running experiments with its missions, such as the time limit of a time constrained mission or the frequency of thruster failure. These can be found in the [mission_config.yaml](https://github.com/kas-lab/suave/blob/main/suave_missions/config/mission_config.yaml) file. **Note:** When you change the mission_config file, you need to rebuild the suave_ws with `colcon build --symlink-install`

**Note:** Before starting the simulation or the ros nodes, remember that you have to source SUAVE's workspace. If you are using the dockerized version this is already done for you, therefore sourcing the workspace is not necessary.

Navigate to the workspace and source it:
```Bash
cd ~/suave_ws/
source install/setup.bash
```

With SUAVE configured and sourced, start ArduSub, the simulation, and the SUAVE's nodes with the following instructions.

#### Start ArduSub

Run:
```Bash
sim_vehicle.py -L RATBeach -v ArduSub --model=JSON --console
```

#### Start the simulation

Run:
```Bash
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

**Note:** It is possible to pass arguments to specify the x and y coordinates of where the UUV spawns, by changing the values. In the above launch command the initial coordinates are set to (-17.0, 2.0).

#### Start SUAVE's nodes

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

'mission_type':
    BT mission label written to metrics
    (default: 'time_constrained_mission')

'result_filename':
    Filename for the mission measured metrics
    available values: any name
    (default: empty; the selected manager uses its metrics default)

'use_action_server':
    For the BT manager, start managed behaviors through ROS 2 actions
    available values: true/false
    (default: false)
```

The arguments can be defined by adding the above arguments with the notation `<name>:=<value>` to the end of the command line.

An example of running with metacontrol and saving to a file called 'measurement_1':

```Bash
ros2 launch suave_missions mission.launch.py adaptation_manager:=metacontrol result_filename:=measurement_1
```

An example of running the BT manager in action-server mode:

```Bash
ros2 launch suave_missions mission.launch.py \
  adaptation_manager:=bt use_action_server:=true
```

## Extending SUAVE and connecting managing subsystems

### Connecting managing subsystems

SUAVE is designed to allow for different managing subsystems to be used, as long as they adhere to the correct ROS 2 interfaces.
SUAVE's ROS2 interfaces are:

1. The `/diagnostics` topic, which is where monitoring information is published. This topic uses the [diagnostic_msgs/DiagnosticArray](https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html) message type
2. The `/task/request` and `/task/cancel` services, which are used to request and cancel tasks, respectively. Both services use the [suave_msgs/Task](https://github.com/kas-lab/suave/blob/main/suave_msgs/srv/Task.srv) service type
3. Three [system_modes](https://github.com/micro-ROS/system_modes) services to change SUAVE's LifeCycle nodes mode. These services use the [system_modes_msgs/ChangeMode](https://github.com/micro-ROS/system_modes/blob/master/system_modes_msgs/srv/ChangeMode.srv) service type:
    1. Service `/f_maintain_motion/change_mode` to change the Maintain Motion node modes
    2. Service `/f_generate_search_path/change_mode` to change the Generate Search Path node modes
    3. Service `/f_follow_pipeline/change_mode` to change the Follow Pipeline node modes

Thus, to connect a different managing subsystem to SUAVE, it must subscribe to `/diagnostics` to get monitoring information, send adaptation goals (task) requests via `/task/request` and `/task/cancel`, and send reconfiguration requests via `/f_maintain_motion/change_mode`, `/f_generate_search_path/change_mode`, or `/f_follow_pipeline/change_mode`.


In order to use the new managing subsystem with the launchfile [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_missions/launch/mission.launch.py) as explained in the [run suave](#run-suave) section, a new launchfile must be created for the new managing subsystem (check [suave_metacontrol.launch.py](https://github.com/kas-lab/suave/blob/main/suave_metacontrol/launch/suave_metacontrol.launch.py) for an example), and the new launch file must be included in the [mission.launch.py](https://github.com/kas-lab/suave/blob/main/suave_missions/launch/mission.launch.py) file.


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

### Extend SUAVE

To extend SUAVE with new functionalities, it is only required to add new LifeCycle nodes that implement the new functionalities (check [spiral_search_lc.py](https://github.com/kas-lab/suave/blob/main/suave/suave/spiral_search_lc.py) for an example), and add its different modes to the [system_modes](https://github.com/micro-ROS/system_modes) configuration file [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml). Note, that if you create a new configuration file, you should replace the [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml) path with the new file path.

## Troubleshooting

List of known problems and their solutions. If you find new problems and find a solution for it, please consider contributing to this section.

#### MAVROS and ArduSub integration problems:

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

To find the latest version of ArduSub go to the [ardupilot repo](https://github.com/ArduPilot/ardupilot) and look for the newest branch of ArduSub. This repository is tested with [ArduSub commit `571e8c7`](https://github.com/ArduPilot/ardupilot/tree/571e8c7bd3793fce1bc5184a2f6586feb8a616e5). Follow the [install ArduSub](#install-ardusub) instructions with the selected commit or branch.

**Update MAVROS:**
To update MAVROS, change its version in [suave.repos](https://github.com/kas-lab/suave/blob/main/suave.repos) to the desired [MAVROS](https://github.com/mavlink/mavros) version or branch, then pull the workspace repositories:

```Bash
cd ~/suave_ws/
vcs pull src
```

Alternatively, update MAVROS manually:

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

## Related repository

[REMARO Summer School Delft 2022](https://github.com/remaro-network/tudelft_hackathon) - Underwater robotics hackathon

## Citation

If you find this repository useful, please consider citing the [SUAVE paper](https://ieeexplore.ieee.org/abstract/document/10173938/):

```
@INPROCEEDINGS{10173938,
  author={Silva, Gustavo Rezende and Päßler, Juliane and Zwanepol, Jeroen and Alberts, Elvin and Tarifa, S. Lizeth Tapia and Gerostathopoulos, Ilias and Johnsen, Einar Broch and Corbato, Carlos Hernández},
  booktitle={2023 IEEE/ACM 18th Symposium on Software Engineering for Adaptive and Self-Managing Systems (SEAMS)},
  title={SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles},
  year={2023},
  volume={},
  number={},
  pages={181-187},
  doi={10.1109/SEAMS59076.2023.00031}}
```

## Acknowledgments

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Please visit [our website](https://remaro.eu/) for more information on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
