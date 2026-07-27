# Install SUAVE locally
To install the exemplar locally, you have to [install Gazebo Harmonic](#install-gazebo-harmonic), [install ROS2 Humble](#install-ros2-humble), [install ROS_GZ](#install-ros_gz), [install ArduSub](#install-ardusub), [install the ArduSub plugin](#install-ardusub-plugin), and finally [install the SUAVE workspace](#install-suave-workspace).

## Install Gazebo Harmonic

Follow the [official instructions](https://gazebosim.org/docs/harmonic/install_ubuntu/#binary-installation-on-ubuntu) for installing Gazebo Harmonic.

## Install ROS2 Humble

Follow the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for installing ROS2 Humble.

## Install ROS_GZ

```Bash
sudo apt install ros-humble-ros-gzharmonic
```

## Install ArduSub
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

## Install ArduSub plugin

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

## Install SUAVE workspace

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
to reproduce it is the archived container image:

```Bash
docker run -it --shm-size=512m -p 6901:6901 -e VNC_PW=password \
  --security-opt seccomp=unconfined ghcr.io/kas-lab/suave:seams2023
```

Use the `main` or release-tag instructions above for current development.

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
