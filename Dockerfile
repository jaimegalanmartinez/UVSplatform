#Base image - Ubuntu 20.04
FROM ubuntu:20.04
#FROM cyberbotics/webots:R2022a-ubuntu20.04

#Disable Prompt During packages installation
ARG DEBIAN_FRONTEND=noninteractive

#Change shell to bash in order to use source command
SHELL ["/bin/bash", "-c"]

# https://github.com/NVIDIA/nvidia-docker/issues/1632#issuecomment-1112667716
# https://forums.developer.nvidia.com/t/notice-cuda-linux-repository-key-rotation/212772
# To solve GPG error:

# Remove outdated signing key
#RUN apt-key del 7fa2af80
# Install new signing key
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb && dpkg -i cuda-keyring_1.0-1_all.deb
#RUN sed -i '/developer\.download\.nvidia\.com\/compute\/cuda\/repos/d' /etc/apt/sources.list
# Fix duplicate .list entries
#RUN rm /etc/apt/sources.list.d/cuda.list

# Install new key

#Update Ubuntu SW repository
RUN apt-get update && apt-get install apt-utils -y


WORKDIR /
#Install git
RUN apt-get update && \
    apt-get install git -y

#Install build essentials (CMAKE)
RUN apt-get update && \
    apt-get install build-essential -y
    
#Install ROS2 Galactic
#Set locale
RUN locale && \
    apt-get update && apt-get install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale
#Setup sources
RUN apt-cache policy | grep universe && \
    apt-get install software-properties-common -y && \
    add-apt-repository universe && \
    apt-get update && apt-get install curl gnupg lsb-release -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | \ 
tee /etc/apt/sources.list.d/ros2.list > /dev/null

#Install ROS2 packages - ROS2 Galactic
RUN apt-get update && \
    apt-get install ros-galactic-ros-base -y

#Install rosdep and colcon
RUN apt-get update && apt-get install -y python3-rosdep && \
    rosdep init && rosdep update && \
    apt-get -y install python3-colcon-common-extensions

#Create ROS2 Workspace
RUN mkdir -p ros2_ws/src

#Install webots_ros2 package
# Retrieve the sources
#RUN git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git ros2_ws/src/webots_ros2

# Install dependencies
RUN rosdep update && \
    rosdep install -y --from-paths ros2_ws/src --ignore-src --rosdistro galactic

# Set working directory to /ros2_ws
WORKDIR /ros2_ws

#Environment setup and build ros2 workspace (ros2_ws)
RUN source /opt/ros/galactic/setup.bash && colcon build && \
    echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

#Copy uvs_pkg and uvs_fleet_manager to src folder
ADD ros2_modules/uvs_pkg src/uvs_pkg
ADD ros2_modules/uvs_fleet_manager src/uvs_fleet_manager

# Build uvs_pkg
RUN source /opt/ros/galactic/setup.bash && colcon build --packages-select uvs_pkg uvs_fleet_manager
# Build uvs_fleet_manager
#RUN source /opt/ros/galactic/setup.bash && colcon build --packages-select uvs_fleet_manager
# Local environment setup
RUN . install/local_setup.bash


