#!/bin/bash
# stop the program to run ros nodes

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.131
export ROS_MASTER_URI=http://192.168.1.21:11311


var1=$(pgrep tx2_local_syste)
kill $var1 > /dev/null

rosnode kill /local_serial_node /local_system /local_serial
sleep 3
rosnode kill /local_serial_node /local_system /local_serial