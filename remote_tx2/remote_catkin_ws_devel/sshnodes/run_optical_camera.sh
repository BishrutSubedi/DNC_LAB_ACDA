#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.130
export ROS_MASTER_URI=http://192.168.1.21:11311

var1=$1


if [ $var1 -eq 1 ]
then 
	echo start optical camera 
	roslaunch jetson_csi_cam  jetson_csi_cam.launch width:=640 height:=480 fps:=10 
else
	echo Do not run optical camera 
fi	
	

