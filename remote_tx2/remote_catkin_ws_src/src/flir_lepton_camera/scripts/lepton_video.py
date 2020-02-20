#!/usr/bin/env python

import rospy
import roslib
import sys
import numpy as np
import cv2
from pylepton import Lepton
from std_msgs.msg import String
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError


def capture(flip_v = False, device = "/dev/spidev3.0") :
  with Lepton(device) as l:
    a,_ = l.capture()
  if flip_v:
    cv2.flip(a,0,a)
  cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX)
  np.right_shift(a, 8, a)
  return np.uint8(a)



bridge=CvBridge()

if __name__=='__main__':
	
	#cv2.namedWindow('image',cv2.WINDOW_NORMAL)
	pub1 = rospy.Publisher("infrared_img", Image, queue_size=1)
	rospy.init_node('infrared_video', anonymous=False)
   # pub=rospy.Publisher("flir_img", Image)
    #rospy.init_node("infared_image", anonymous= True)
    
	#cv2.resizeWindow('image',100,100)
	while  not rospy.is_shutdown():
		image=capture()
		image=cv2.applyColorMap(image,cv2.COLORMAP_JET)
                image=cv2.resize(image,(480,360))
		image_message= bridge.cv2_to_imgmsg(image, "bgr8")
		#image_message= bridge.cv2_to_imgmsg(image, encoding="passthrough")
        
		pub1.publish(image_message)
        #cv2.imshow("image",image)
        #if cv2.waitKey(1) & 0XFF==ord('q'):
		#		break
