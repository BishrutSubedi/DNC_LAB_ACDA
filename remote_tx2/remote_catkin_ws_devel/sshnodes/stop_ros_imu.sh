#!/bin/sh

#!/bin/bash
# stop the program to run ros nodes
source /opt/ros/kinetic/setup.bash
source /home/nvidia/catkin_ws/devel/setup.bash
export ROS_IP=192.168.1.130
export ROS_MASTER_URI=http://192.168.1.21:11311

(var1=$(pgrep imu_calibration)
kill $var1 > /dev/null) &
(sleep 3
rosnode kill /remote_serial
)