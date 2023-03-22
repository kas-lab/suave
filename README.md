# SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles
This repository contains the exemplar SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar), an exemplar focussing on an underwater robot searching for a pipeline and following it.

It clearly separates between the mananged subsystem, implementing the basic functionalities of the robot, and the managing system that implements the adaptation logic. This ensures that this exemplar can be reused with several different managing subsystems, providing that they satisfy the necessary [requirements](#requirements-for-a-managing-subsystem). The usability of the exemplar is showcased with [MROS2](https://github.com/meta-control/mc_mros_reasoner), the implementation of the self-adaptation framework [Metacontrol](https://research.tudelft.nl/en/publications/model-based-self-awareness-patterns-for-autonomy).

The exemplar can either be [run through Docker](#use-the-exemplar-with-docker) or [installed locally](#install-the-exemplar-locally) to [run it](#run-the-exemplar).

## Navigate the README
- [Use the exemplar with Docker](https://github.com/kas-lab/suave#use-the-exemplar-with-docker)
- [Install the exemplar locally](https://github.com/kas-lab/suave#install-the-exemplar-locally)
- [Install the SUAVE workspace](https://github.com/kas-lab/suave#install-the-suave-workspace)
- [Run the exemplar](https://github.com/kas-lab/suave#run-the-exemplar)
- [Extending and connecting managing subsystems](https://github.com/kas-lab/suave#extending-and-connecting-managing-subsystems)
- [Acknowledgments](https://github.com/kas-lab/suave#acknowledgments)

## Use the exemplar with Docker

You can pull and run the exemplar as a Docker container using the following command. Keep in mind you need to have [Docker](https://docs.docker.com/get-docker/) installed on your computer and running.

In a terminal on your computer run:
```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined egalberts/suave:1.0
```

Optionally you can add the parameter `-v <absolute_path_host_compute>:/home/kasm-user/suave/results` to save the results into your computer, replace `<absolute_path_host_compute>` with the absolute path of where you want the data to be saved in your computer, e.g:

```Bash
docker run -it --shm-size=512m -v $HOME/suave_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password egalberts/suave:1.0
```

Once the container is up and running, you can interface with it through your web browser. The container will be hosted locally at the port specified, in this case 6901. So in your browser, go to
`http://localhost:6901`.

A dialog will request a username and password, these are shown below, with the password being specifiable in the run command.

 - **User** : `kasm_user`
 - **Password**: `password`

Now you can proceed to [run the exemplar](#run-the-exemplar).

### Build Docker images locally
As it stands right now due to a bug in the newer versions of Gazebo Garden, newly built images will not work. In the future when this is resolved we will update the README to reflect how this can be done.
<!-- Should you want to make some changes to the Docker container, you can also choose to build it locally. In the provided Docker folder, the `build_images.sh` script can be run to build the image locally. The built image can then be run with the same command afterwards. Instead of pulling the image from Dockerhub, it will use the local image. -->

## Install the exemplar locally
To install the exemplar locally, you have to [install Gazebo Garden](#install-gazebo-garden), [install ROS2 Humble](#install-ros2-humble), [install ArduSub](#install-ardusub), [install the ArduSub plugin](#install-ardusub_plugin), and finally [install the SUAVE workspace](#install-suave-workspace).

#### Install Gazebo Garden

Follow the [official instructions](https://gazebosim.org/docs/garden/install_ubuntu) for installing Gazebo Garden.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble.

#### Install ArduSub
ArduSub is a subproject within ArduPilot for piloting underwater vehicles.

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions. This repo was tested with [this ArduPilot commit](https://github.com/ArduPilot/ardupilot/tree/9f1c4df5e744d58d3089671926bb964c924b2090) and [mavros 2.4.0](https://github.com/mavlink/mavros/tree/9f1c4df). Unfortunately, at least at the time of writing this README, the releases available in Ubuntu 22.04 do not match.

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 9f1c4df
git submodule update --init --recursive
```

Note that the script used to install prerequisites available for this
version of ArduSub does not work in Ubuntu 22.04. Therefore, you need to replace them before
running ArduSub. To install the ArduPilot prerequisites, do the following.

```Bash
cd ~/ardupilot
cd Tools/environment_install/
rm install-prereqs-ubuntu.sh
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/environment_install/install-prereqs-ubuntu.sh
cd ~/ardupilot
chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

To test if the installation worked, run:

```Bash
sim_vehicle.py -v ArduSub -L RATBeach --console --map
```
ArduPilot SITL should open and a console plus a map should appear.

**Troubleshooting:**
If you have problems with the `install-prereqs-ubuntu.sh` script, try to install the dependencies manually with the following commands.

```Bash
pip3 install --user -U future lxml pymavlink MAVProxy pexpect flake8 geocoder empy dronecan pygame intelhex
```

```Bash
sudo apt-get --assume-yes install build-essential ccache g++ gawk git make wget python-is-python3 libtool libxml2-dev libxslt1-dev python3-dev python3-pip python3-setuptools python3-numpy python3-pyparsing python3-psutil xterm python3-matplotlib python3-serial python3-scipy python3-opencv libcsfml-dev libcsfml-audio2.5 libcsfml-dev libcsfml-graphics2.5 libcsfml-network2.5 libcsfml-system2.5 libcsfml-window2.5 libsfml-audio2.5 libsfml-dev libsfml-graphics2.5 libsfml-network2.5 libsfml-system2.5 libsfml-window2.5 python3-yaml libpython3-stdlib python3-wxgtk4.0 fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libtool-bin g++-arm-linux-gnueabihf lcov gcovr
```

#### Install the ArduSub plugin

Install the dependencies:

```Bash
sudo apt install libgz-sim7-dev rapidjson-dev
```

Clone and build the repository:

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
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
wget https://raw.githubusercontent.com/kas-lab/suave/main/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive
```
**SEAMS2023:** If you want to get the version submitted to SEAMS 2023 instead of the most updated version get the following dependencies instead:

```Bash
wget https://raw.githubusercontent.com/kas-lab/suave/35d482b85190a15894461bd17748e3e5a36f576b/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive
```

Install the dependencies:
```Bash
source /opt/ros/humble/setup.bash
cd ~/suave_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Before building the `ros_gz` package (one of the dependencies), you need to export the gazebo version:

```
export GZ_VERSION="garden"
```
You can also add this to your `~/.bashrc` to make this process easier.

Build the project:
```Bash
cd ~/suave_ws/
colcon build --symlink-install
```

If you have memory problems while building the package, run the following command instead, it is slower but uses less memory:

```Bash
colcon build --symlink-install --executor sequential --parallel-workers 1
```

Now you can proceed to [run the exemplar](#run-the-exemplar).

## Run the exemplar
### Trying it out!
If you simply want to try out the exemplar while running the docker image, simply enter the following commands in a terminal:
```Bash
cd ~/suave_ws/
./example_run.sh
```
Within a couple of minutes, some new terminals should open as well as the Gazebo simulator.
A default mission is executed of inspecting the pipeline with a time limit.
To follow the robot as it progesses along its mission make sure to right click and follow it in the entity tree of Gazebo as shown below:
![BLUEROV Follow](https://github.com/kas-lab/suave/blob/652db0676ec2995c4cc0653ef5de0fc49edd00ac/docker/follow_bluerov.PNG)

**Please note**: It can take a little while for the robot to get moving, it is an issue we are aware of. Once it does get a move on you should see it perform its mission for about 5 minutes.

### Full Runner
To run the exemplar with the runner from within the docker image, make sure you are in the ~/suave_ws/ directory and simply run:


Without gui:
```
./runner false
```

With gui:
```
./runner true
```

To run it from a local installation, navigate to the folder `docker/runner/` first:
```
cd docker/runner/
./runner true
```

### Without runner

ArduSub: To start the autopiloting software for the simulated AUV run the following command in a terminal.
```
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```
Configuring SUAVE:
SUAVE has a number of parameters that may be of interest when running experiments with its missions, such as the time limit of a time constrained mission or the frequency of thruster failure. These can be found in the [mission_config.yaml](https://github.com/kas-lab/suave/blob/main/suave_missions/config/mission_config.yaml) file. TODO: explain whether you need to rebuild or not after changing them.

Start the simulation:
Note: You should make sure to source the install before running ros2 commands. If you are using the dockerized version this is already done for you, therefore sourcing the workspace is not necessary.
To start the Gazebo simulator with the simulated AUV and surrounding underwater pipeline scenario run the following commands in a new terminal.

Navigate to the workspace and source it:
```Bash
cd ~/suave_ws/
source install/setup.bash
```

Launch the simulation environment:
```
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

**Note:** It is possible to pass arguments to specify the x and y coordinates of where the UUV spawns, by changing the values. In the above launch command the initial coordinates are set to (-17.0, 2.0).

### Start the Mission
Now it is possible to start the mission. Select the desired type of missions through the command line arguments. The mission results will be saved in the path specified in the mission_config.yaml file.

Launching the mission file without launch arguments will start a time-constrained mission without a managing subsystem. This can be done by running the following command:

```Bash
ros2 launch suave_missions mission.launch.py
```

The arguments can be defined by adding the below arguments with the notation `<name>:=<value>` to the end of the command line. The available arguments are:
```
'adaptation_manager':
    Managing subsystem to be used
    available values: none/metacontrol/random
    (default: 'none')

'mission_type':
    Type of mission to be executed
    available values: time_constrained_mission/const_dist_mission
    (default: 'time_constrained_mission')

'result_filename':
    Filename for the mission measured metrics
    available values: any name
    (default: 'time_constrained_mission_results')
```

An example of running the constant distance mission with metacontrol saving to a file called 'measurement_1' is as follows:

```Bash
ros2 launch suave_missions mission.launch.py adaptation_manager:=metacontrol mission_type:=const_dist_mission result_filename:=measurement_1
```

## Extending and connecting managing subsystems

The interface for interacting with the managed subsystem is handle by the ROS2 system modes package. For easy integration with another managing subsystem, this package can be used to apply reconfiguration. The nodes which handle searching for and following the pipeline, as well as motion control are implemented using Lifecycle Nodes. This allows for these nodes to be reconfigured using the system modes package.

The adaptation scenarios need to be described in a yaml file such as done in [suave_modes.yaml](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml) within the same folder. This file should then be added in the [system_modes.launch.py](https://github.com/kas-lab/suave/blob/main/suave/launch/system_modes.launch.py) file, replacing the suave_modes.yaml.

The adaptation logic needs to be implemented as a separate node. Adaptation is done by calling the system_modes services for each Lifecycle Node. The [random_reasoner.py](https://github.com/kas-lab/suave/blob/main/suave_metacontrol/suave_metacontrol/random_reasoner.py) node can be used as a reference for applying the adaptation logic.

In order to launch the new managing subsystem using mission.launch.py as explained in the [run the exemplar](#run-the-exemplar) section, the launch argument needs to be linked to the executable of the managing subsystem node by replacing \[new_managing_subsystem\] with the given name:
```
[new_managing_subsystem]_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        '[new_managing_subsystem].launch.py')

[new_managing_subsystem]_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([new_managing_subsystem]_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager',
                    '[new_managing_subsystem]'))
```

## Related repository

[REMARO Summer School Delft 2022](https://github.com/remaro-network/tudelft_hackathon) - Underwater robotics hackathon


## Acknowledgments

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Please visit [our website](https://remaro.eu/) for more information on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
