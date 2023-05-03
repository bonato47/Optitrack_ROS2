# Handle ros distro
ARG ROS_DISTRO=galactic

FROM ghcr.io/aica-technology/ros2-ws:${ROS_DISTRO}

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Setup git identity
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

# Setup python version for noetic
RUN sudo apt update
RUN sudo apt install python-is-python3

### Add a few tools
RUN sudo apt-get update && sudo apt-get install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    netbase \
    && sudo apt-get upgrade -y && sudo apt-get clean
#RUN sudo apt update && sudo apt install -y ros-core-dev -y
# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi
    
# Install Optitrack_ROS2
WORKDIR /home/${USER}
RUN git clone --depth 1 https://github.com/bonato47/Optitrack_ROS2.git
RUN cp -R Optitrack_ROS2/src/ros1_ws/ .
RUN cp -R Optitrack_ROS2/src/ros2_ws/ .

<<<<<<< HEAD
=======
# Install ros_bridge
WORKDIR /home/${USER}/ros2_ws/src
RUN git clone https://github.com/ros2/ros1_bridge.git
WORKDIR /home/${USER}/ros2_ws/src/ros1_bridge
RUN git checkout -b ${ROS_DISTRO}

# Install subsciiber


>>>>>>> parent of 0659941 (dock)
#Cleaning the workspace
RUN rm -r /home/${USER}/Optitrack_ROS2

### Add environement variables to bashrc
WORKDIR /home/${USER}

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

### Add environement variables to bashrc

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
RUN sudo apt install curl 
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sudo apt update
RUN sudo apt install ros-noetic-desktop-full -y
RUN sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential 
RUN sudo apt-get install ros-noetic-vrpn-client-ros -y

### Build ros workspace
WORKDIR /home/${USER}/ros1_ws/
RUN source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && rosdep install roscpp
RUN source /opt/ros/noetic/setup.bash && catkin_make;
#sudo -H apt-get install -y ros-galactic-demo-nodes-cpp

### Build ros2 workspace
WORKDIR /home/${USER}/ros2_ws/
RUN source /opt/ros/noetic/setup.bash && source /opt/ros/galactic/setup.bash && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure && colcon build;
WORKDIR /home/${USER}

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean
