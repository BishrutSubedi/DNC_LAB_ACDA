#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.130
#export ROS_MASTER_URI=http://192.168.1.121:11351
#export ROS_MASTER_URI=http://192.168.1.21:11311
#(rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200)&
(roslaunch /home/nvidia/catkin_ws/devel/sshnodes/remote_serial.launch) &
(sleep 10
rosrun imu_calibration imu_calibration_node
rosnode kill /remote_serial_node
killall -9 roscore
sleep 3
killall -9 rosmaster

)