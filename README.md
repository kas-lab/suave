# SUAVE: Self-adaptive Underwater Autonomous Vehicle Exemplar
Underwater pipeline inspection self-adaptive exemplar

## Run through docker

You can pull and run the exemplar as a docker container using the following command. Keep in mind you need to have docker installed on your computer and running (https://docs.docker.com/get-docker/).

```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password egalberts/suave:dev
```

Once it is up and running, you can interface with the container through your web browser. The container will be hosted locally at the port specified, in this case 6901.
`https://<IP>:6901`

You may receive a warning about the invalidity of the https certification. This is a known issue and can be safely ignored by just advancing through the warning.Then a dialog will request a username and password, these are shown below, with the password being specifiable in the run command.

 - **User** : `kasm_user`
 - **Password**: `password`

### Build docker images locally
Should you want to make some changes to the docker container, you can also choose to build it locally. In the docker folder provided the build_images.sh script can be run to build the image locally which can then be run with the same command afterwards. Instead of pulling the image from dockerhub it will use the local image instead.

## Install locally

#### Install Gazebo Garden

Follow the [official instructions](https://gazebosim.org/docs/garden/install_ubuntu) for installing Gazebo Garden.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble.

#### Install ardusub

Instructions can be found [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions. This repo was tested with [ardupilot in commit c623ae8](https://github.com/ArduPilot/ardupilot/tree/9f1c4df5e744d58d3089671926bb964c924b2090) and [mavros 2.4.0](https://github.com/mavlink/mavros/tree/10569e626a36d8c69fc78749bb83c112a00e2be8). Unfortunately, at least at the time of writing this README, the releases available in Ubuntu 22.04 do not match.

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout 9f1c4df
git submodule update --init --recursive
```

Unfortunately, the script used to install prerequisites available in this
version of ardusub don't work in Ubuntu 22.04. So you need to replace it before
running it. Install ardupilot prerequisites:

```Bash
cd ardupilot
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

Ardupilot SITL should open and a console plus a map should appear.

**Troubleshoot**
If you have problems with the `install-prereqs-ubuntu.sh` script try to install the dependencies manually with the following commands.

```Bash
pip3 install --user -U future lxml pymavlink MAVProxy pexpect flake8 geocoder empy dronecan pygame intelhex
```

```Bash
sudo apt-get --assume-yes install build-essential ccache g++ gawk git make wget python-is-python3 libtool libxml2-dev libxslt1-dev python3-dev python3-pip python3-setuptools python3-numpy python3-pyparsing python3-psutil xterm python3-matplotlib python3-serial python3-scipy python3-opencv libcsfml-dev libcsfml-audio2.5 libcsfml-dev libcsfml-graphics2.5 libcsfml-network2.5 libcsfml-system2.5 libcsfml-window2.5 libsfml-audio2.5 libsfml-dev libsfml-graphics2.5 libsfml-network2.5 libsfml-system2.5 libsfml-window2.5 python3-yaml libpython3-stdlib python3-wxgtk4.0 fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libtool-bin g++-arm-linux-gnueabihf lcov gcovr
```

#### Install ardusub_plugin

Install dependencies:

```Bash
sudo apt install libgz-sim7-dev rapidjson-dev
```

Clone and build repo:

```Bash
cd ~/
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Add required paths:

Assuming that you have clone the repository in `$HOME/ardupilot_gazebo`:
```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with source ~/.bashrc

More info about the plugin can be found in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/)

### Install suave workspace

Create workspace and download required repositories:
```Bash
mkdir -p ~/suave_ws/src/
cd ~/suave_ws/
wget https://raw.githubusercontent.com/kas-lab/suave/main/suave/suave.rosinstall
vcs import src < suave.rosinstall --recursive
```

Add required paths:
```Bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/bluerov2_ignition/models:$HOME/suave_ws/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/suave_ws/src/remaro_worlds/models:$HOME/suave_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Install deps:
```Bash
source /opt/ros/humble/setup.bash
cd ~/suave_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Build project:
```Bash
cd ~/suave_ws/
colcon build --symlink-install
```

## Run locally

ArduSub:
```
sim_vehicle.py -L RATBeach -v ArduSub  --model=JSON --console
```

Start Simulation:
Note: You should make sure to source the install before running ros2 commands. e.g. source suave_ws/install/setup.bash
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

Start Mission (select one of the missions to run):

Time constrained mission:
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
Time constrained mission with no adaptation:
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

Constant distance mission:
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

Constant distance mission with no adaptation:
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

Mission results will be save in `<result_path>/<result_filename>.csv`
