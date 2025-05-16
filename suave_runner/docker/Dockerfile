# Use the official ROS 2 Humble base image with Gazebo Fortress pre-installed
FROM osrf/ros:humble-desktop-full

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN bash -c 'curl https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list --output /etc/ros/rosdep/sources.list.d/00-gazebo.list'
ENV GZ_VERSION=harmonic 

# Update and install additional dependencies if needed
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    python3-vcstool \
    python3-rosdep \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    vim \
    python3-pip \
    libgz-sim8-dev \ 
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    xvfb \
    htop \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

    
###Add the USER env var
RUN groupadd -g 1000 ubuntu-user \
    && adduser --disabled-password --gid 1000 --uid 1000 --gecos '' ubuntu-user \
    && adduser ubuntu-user sudo

RUN echo 'ubuntu-user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers
ENV HOME=/home/ubuntu-user
USER ubuntu-user
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR $HOME
# Install ArduSub
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR $HOME/ardupilot
RUN git checkout 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae
RUN git submodule update --init --recursive
RUN rm $HOME/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
RUN wget -P $HOME/ardupilot/Tools/environment_install/ https://raw.githubusercontent.com/ArduPilot/ardupilot/c623ae8b82db4d7e195f4b757e2ae5d049f941e5/Tools/environment_install/install-prereqs-ubuntu.sh
RUN chmod +x $HOME/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh
RUN USER=ubuntu-user Tools/environment_install/install-prereqs-ubuntu.sh -y

RUN sudo pip3 install -U mavproxy PyYAML

ENV PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH
ENV PATH=$PATH:$HOME/ardupilot/Tools/autotest
ENV PATH=/usr/lib/ccache:$PATH

RUN ["/bin/bash","-c","cd $HOME/ardupilot && ./waf configure && make sub"]

# cd ~/ardupilot
# cd Tools/environment_install/
# rm install-prereqs-ubuntu.sh
# wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/environment_install/install-prereqs-ubuntu.sh
# cd ~/ardupilot
# chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
# Tools/environment_install/install-prereqs-ubuntu.sh -y
# . ~/.profile

WORKDIR $HOME
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo && \
    cd ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j4

# RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> /etc/bash.bashrc && \
#     echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> /etc/bash.bashrc
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
ENV GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

WORKDIR $HOME
RUN mkdir -p suave_ws/src
WORKDIR $HOME/suave_ws/src
RUN git clone https://github.com/kas-lab/suave.git -b new_runner

WORKDIR $HOME/suave_ws
RUN vcs import src < src/suave/suave.rosinstall --recursive

# && rosdep resolve gz-harmonic \
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/"]

RUN ["/bin/bash", "-c", "sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh"]

## There is a bug with the default setuptools and packaging versions
RUN USER=ubuntu-user python3 -m pip install setuptools==75.8.2  packaging==24.1 empy==3.3.4
RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install \
    && echo 'source ~/suave_ws/install/setup.bash' >> ~/.bashrc"]
    
RUN sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/