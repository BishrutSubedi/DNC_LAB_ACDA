#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.130
export ROS_MASTER_URI=http://192.168.1.21:11311

echo run infrared camera node
var1=$1
if [ $var1 -eq 1 ]
then
	
	rosrun flir_lepton_camera lepton_video.py  
else 
	echo Do not run infrared camera
fi

