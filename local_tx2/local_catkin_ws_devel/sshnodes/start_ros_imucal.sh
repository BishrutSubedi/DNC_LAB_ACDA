#!/bin/bash

# run IMU calibrateion ros nodes
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.131
export ROS_MASTER_URI=http://192.168.1.21:11311
(rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200)&
(sleep 10
rosrun imu_calibration imu_calibration_node
sleep 5
rosnode kill /serial_node)
