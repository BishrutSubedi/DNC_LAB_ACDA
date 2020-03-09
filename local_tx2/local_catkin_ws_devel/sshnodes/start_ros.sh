#!/bin/bash 
#start the ros node of directional antenna system
var1=$1 
source /opt/ros/kinetic/setup.bash 
source /home/nvidia/catkin_ws/devel/setup.bash 
export ROS_IP=192.168.1.131
export ROS_MASTER_URI=http://192.168.1.21:11311
#/home/ubuntu/Xsens_compass_calibration/build/main
(rosrun start_ros_serial start_serial.py)&
(sleep 5
rosrun tx2_local_system tx2_local_system_node $var1)
