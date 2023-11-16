# Optitrack_ROS1

## Authors/Maintainers
- Tristan Bonato: tristan.bonato@epfl.ch, @bonato47 on github.

this code tun vrpn_client_ros to connect with optitrack and transform an object to its base.

Go in the roslaunch to put the good ip and the good name of the object and the base.
server:=128.178.145.104
id_base:=  ... 
id_object:= ...



Terminal #1
cd Optitrack_ROS2/ros1_ws/docker
bash build_docker.sh (if not already done)
bash start_optitrack.sh 



