This Docker is based on the iiwa_ros git hub repo :https://github.com/epfl-lasa/iiwa_ros

## Authors/Maintainers
- Tristan Bonato: tristan.bonato@epfl.ch, @bonato47 on github.



Terminal #1
cd Optitrack_ROS2/docker
bash build_docker 
bash start_docker server
bash start_docker connect
cd ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch opti_track_ros_interface vrpn_optitrack.launch 

Terminal #2
cd Optitrack_ROS2/docker
bash start_docker connect
cd ros2_ws
ROS_DOMAIN_ID=99 ros2 run ros1_bridge dynamic_bridge

Terminal #3
cd Optitrack_ROS2/docker
bash start_docker connect
cd ros2_ws
ROS_DOMAIN_ID=99 ros2 run cpp_pub_sub listener
