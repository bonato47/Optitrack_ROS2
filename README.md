# Optitrack_ROS2

This Docker is based on the iiwa_ros git hub repo :https://github.com/epfl-lasa/iiwa_ros

## Authors/Maintainers
- Tristan Bonato: tristan.bonato@epfl.ch, @bonato47 on github.

To bind rosbridge we create a subscirber with the desired topics. You can change the name of the topic in ros2_ws/src/cpp_pubsub/



Terminal #1
cd Optitrack_ROS2/docker
bash build_docker.sh 
bash start_docker.sh server
bash start_docker.sh connect
cd ros1_ws
source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch talk opti_main.launch 

Terminal #2
cd Optitrack_ROS2/docker
bash start_docker.sh connect
cd ros2_ws
ROS_DOMAIN_ID=99 ros2 run ros1_bridge dynamic_bridge

