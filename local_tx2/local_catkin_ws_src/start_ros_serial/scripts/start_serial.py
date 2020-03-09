#!/usr/bin/env python
import  rospy
import os
import sys
from std_msgs.msg import String
global i
i=1
def callback(data):
    global i
    if i==1:
       rospy.loginfo(rospy.get_caller_id() + "I heard %s, &d", data.data)
       i=i+1
       os.system("roslaunch /home/nvidia/catkin_ws/devel/sshnodes/local_serial.launch")
       sys.exit()
def listener():
    rospy.init_node('local_serial', anonymous=False)
    rospy.Subscriber("local_ros_serial", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
