#!/usr/bin/env python

import rospy
import roslib
import sys
import numpy as np
import cv2
from pylepton import Lepton3
import time
# import Lepton3
# import Lepton
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def capture(flip_v=False, device="/dev/spidev3.0"):
    with Lepton3.Lepton3(device) as l:
        a,_ =l.capture()
    if flip_v:
        cv2.flip(a, 0, a)
    cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(a, 8, a)
    return np.uint8(a)


bridge = CvBridge()

if __name__ == '__main__':

    # cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    pub1 = rospy.Publisher("chatterimg", Image, queue_size=1)
    rospy.init_node('talkerimg', anonymous=True)
    # pub=rospy.Publisher("flir_img", Image)
    # rospy.init_node("infared_image", anonymous= True)

    # cv2.resizeWindow('image',100,100)
    l=Lepton3.Lepton3()


    while not rospy.is_shutdown():
        image = capture()

        # a, _ =l.capture()
        # if flip_v:
        #     cv2.flip(a, 0, a)
        # cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX)
        # np.right_shift(a, 8, a)
        # image=np.uint8(a)

        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
        #image=cv2.resize (image, (400,300))
        image_message = bridge.cv2_to_imgmsg(image, "bgr8")
        # image_message= bridge.cv2_to_imgmsg(image, encoding="passthrough")

        pub1.publish(image_message)
        time.sleep(0.01)
        #cv2.imshow("image",image)
        if cv2.waitKey(1) & 0XFF==ord('q'):
             break
