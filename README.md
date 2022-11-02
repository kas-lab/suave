# pipeline_inspection
Underwater pipeline inspection use case

## Install locally

#### Install Gazebo Garden

Follow the [official instructions](https://gazebosim.org/docs/garden/install_ubuntu) for installing Gazebo Garden.

#### Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble.

#### Install ardusub

Instructions can be found [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

**Disclaimer:**
Problems may occur with different combinations of ArduPilot and MavROS versions. This repo was tested with [ardupilot in commit c623ae8](https://github.com/ArduPilot/ardupilot/tree/c623ae8b82db4d7e195f4b757e2ae5d049f941e5) and [mavros 2.2.0](https://github.com/mavlink/mavros/tree/686bd833e7d6ea5542977178872762dfbec5ed89). Unfortunately, at least at the time of writing this README, the releases available in Ubuntu 22.04 do not match.

```Bash
  cd ~/
  git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
  cd ardupilot
  git checkout c623ae8
```

Don't forget to install ardupilot prerequisites:

```Bash
  cd ardupilot
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

**IMPORTANT:** Clone into the ignition-garden branch:

Install dependencies:

```Bash
  sudo apt install rapidjson-dev libgz-sim7-dev
```

Clone and build repo:

```Bash
  cd ~/
  git clone https://github.com/ArduPilot/ardupilot_gazebo -b ignition-garden
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

More info about the plugin can be found in the [repo](https://github.com/ArduPilot/ardupilot_gazebo/tree/ignition-garden)

### Install pipeline_inspection workspace

Create workspace and download required repositories:
```Bash
$ mkdir -p ~/pipeline_ws/src/
$ cd ~/pipeline_ws/
$ wget https://raw.githubusercontent.com/kas-lab/pipeline_inspection/main/pipeline_inspection.rosinstall
$ vcs import src < pipeline_inspection.rosinstall --recursive
```

Add required paths:
```Bash
$ echo 'export GZ_SIM_RESOURCE_PATH=$HOME/pipeline_ws/src/bluerov2_ignition/models:$HOME/pipeline_ws/src/bluerov2_ignition/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
$ echo 'export GZ_SIM_RESOURCE_PATH=$HOME/pipeline_ws/src/remaro_worlds/models:$HOME/pipeline_ws/src/remaro_worlds/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Install deps:
```Bash
$ source /opt/ros/humble/setup.bash
$ cd ~/pipeline_ws/
$ rosdep install --from-paths src --ignore-src -r -y
```

Build project:
```Bash
$  cd ~/pipeline_ws/
$  colcon build --symlink-install
```
