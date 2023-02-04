# SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles
This repository contains the exemplar SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar), an exemplar focussing on an underwater robot searching for a pipeline and following it.

It clearly separates between the mananged subsystem, implementing the basic functionalities of the robot, and the managing system that implements the adaptation logic. This ensures that this exemplar can be reused with several different managing subsystems, providing that they satisfy the necessary [requirements](#requirements-for-a-managing-subsystem). The usability of the exemplar is showcased with [MROS2](https://github.com/meta-control/mc_mros_reasoner), the implementation of the self-adaptation framework [Metacontrol](https://research.tudelft.nl/en/publications/model-based-self-awareness-patterns-for-autonomy).

The exemplar can either be [run through Docker](#use-the-exemplar-with-docker) or [installed locally](#install-the-exemplar-locally) to [run it](#run-the-exemplar).

## Requirements for a managing subsystem
TODO

## Use the exemplar with Docker

You can pull and run the exemplar as a Docker container using the following command. Keep in mind you need to have [Docker](https://docs.docker.com/get-docker/) installed on your computer and running.

In a terminal on your computer run:
```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password --security-opt seccomp=unconfined rezenders/suave:seams
```

Optionally you can add the parameter `-v <absolute_path_host_compute>:/home/kasm-user/suave/results` to save the results into your computer, replace `<absolute_path_host_compute>` with the absolute path of where you want the data to be saved in your computer, e.g:

```Bash
docker run -it --shm-size=512m -v $HOME/suave_results:/home/kasm-user/suave/results -p 6901:6901 -e VNC_PW=password rezenders/suave:seams
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
Instructions can be found [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux).

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions. This repo was tested with [ArduPilot in commit c623ae8](https://github.com/ArduPilot/ardupilot/tree/9f1c4df5e744d58d3089671926bb964c924b2090) and [mavros 2.4.0](https://github.com/mavlink/mavros/tree/10569e626a36d8c69fc78749bb83c112a00e2be8). Unfortunately, at least at the time of writing this README, the releases available in Ubuntu 22.04 do not match.

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
cd ../../
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
source `~/.bashrc`
```
More info about the plugin can be found in the corresponding [repository](https://github.com/ArduPilot/ardupilot_gazebo/).

### Install the SUAVE workspace

Create the workspace and download the required repositories:
```Bash
mkdir -p ~/suave_ws/src/
cd ~/suave_ws/
wget https://raw.githubusercontent.com/kas-lab/suave/main/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive
```

Add the required paths:
```Bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/bluerov2_ignition/models:$HOME/suave_ws/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/remaro_worlds/models:$HOME/suave_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Install the dependencies:
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

Now you can proceed to [run the exemplar](#run-the-exemplar).

## Run the exemplar
TODO: Update for runner scripts.

ArduSub: To start the autopiloting software for the simulated AUV run the following command in a terminal outside of the installed ROS packages.
```
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

Start the simulation:
Note: You should make sure to source the install before running ros2 commands, e.g., source suave_ws/install/setup.bash. If you are using the dockerized version this is already done for you.
To start the Gazebo simulator with the simulated AUV and surrounding underwater pipeline scenario run the following command in its own terminal.
```
ros2 launch suave simulation.launch.py x:=-17.0 y:=2.0
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'x':
        Initial x coordinate for bluerov2
        (default: '-17.0')

    'y':
        Initial y coordinate for bluerov2
        (default: '2.0')
```

Start all nodes except mission:
```
ros2 launch suave_metacontrol suave_metacontrol.launch.py
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'water_visibility_period':
        Water visibility period in seconds
        (default: '100')

    'water_visibility_min':
        Minimum value for water visibility
        (default: '1.25')

    'water_visibility_max':
        Maximum value for water visibility
        (default: '3.75')

    'water_visibility_sec_shift':
        Water visibility seconds shift to left
        (default: '0.0')

    'thruster_events':
        (thrusterN, failure/recovery, delta time in seconds ), e.g. (1, failure, 50)
        (default: '['(1, failure,30)', '(2, failure,30)']')
```

### Start the Mission
Now it is possible to start the mission. Select one of the missions below. The mission results will be save in `<result_path>/<result_filename>.csv`.

**Time constrained mission:**
The mission is time constrained, the time limit can be set with `time_limit`.
```Bash
ros2 launch suave_metacontrol time_constrained_mission.launch.py time_limit:=300
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'result_path':
        Path to save mission measured metrics
        (default: '~/suave/results')

    'result_filename':
        Filename for the mission measured metrics
        (default: 'time_constrained_mission_results')

    'time_limit':
        Time limit for the mission (seconds)
        (default: '300')
```
**Time constrained mission with no adaptation:** The mission is time constrained and no adaptation is performend. The time limit can be set with `time_limit`.
```Bash
ros2 launch suave time_constrained_mission_no_adapt.launch.py time_limit:=300
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'result_path':
        Path to save mission measured metrics
        (default: '~/suave/results')

    'result_filename':
        Filename for the mission measured metrics
        (default: 'time_constrained_mission_results')

    'time_limit':
        Time limit for the mission (seconds)
        (default: '300')
```

**Constant distance mission:** The mission is to follow the whole pipeline without a time constraint.
```
ros2 launch suave_metacontrol const_distance_mission.launch.py
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'result_path':
        Path to save mission measured metrics
        (default: '~/suave/results')

    'result_filename':
        Filename for the mission measured metrics
        (default: 'const_distance_mission_results')

```

**Constant distance mission with no adaptation:** The mission is to follow the whole pipeline without a time constraint and without adapting the robot.
```
ros2 launch suave const_distance_mission_no_adapt.launch.py
```

Arguments:
```
Arguments (pass arguments as '<name>:=<value>'):

    'result_path':
        Path to save mission measured metrics
        (default: '~/suave/results')

    'result_filename':
        Filename for the mission measured metrics
        (default: 'const_distance_mission_results')

```

## Acknowledgments

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under the Marie Skłodowska-Curie grant agreement No. 956200.

Pleave visit [our website](https://remaro.eu/) for more information on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
