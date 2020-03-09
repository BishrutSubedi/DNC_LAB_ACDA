#!/bin/sh

# stop the program to calbrate the IMU module

(var1=$(pgrep imu_calibration)
kill $var1 > /dev/null) &
(sleep 2
pkill -f serial_node.py)
