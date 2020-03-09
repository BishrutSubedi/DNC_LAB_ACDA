#!/bin/bash
# stop the program to run ros nodes
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.130
export ROS_MASTER_URI=http://192.168.1.21:11311

var1=$(pgrep tx2_remote_syst)
kill $var1 > /dev/null
var2=$(pgrep roslaunch)
kill $var2 > /dev/null

rosnode kill /remote_system
sleep 3
rosnode kill /remote_serial
sleep 3
rosnode kill /remote_system
sleep 3 