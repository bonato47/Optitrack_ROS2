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
    

RUN sudo pip install zmq
RUN sudo pip install scipy
#RUN sudo apt update && sudo apt install -y ros-core-dev -y
# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi    





### Add environement variables to bashrc
WORKDIR /home/${USER}

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi



### Build ros2 workspace
WORKDIR /home/${USER}/ros2_ws/
RUN source /opt/ros/galactic/setup.bash && colcon build;
WORKDIR /home/${USER}

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean
