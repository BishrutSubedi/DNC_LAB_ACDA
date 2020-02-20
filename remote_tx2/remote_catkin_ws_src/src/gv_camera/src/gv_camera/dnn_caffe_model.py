#!/usr/bin/env python3
import cv2

class dnn_caffe:
    print("loading caffe model ...")
    net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt", "MobileNetSSD_deploy.caffemodel")
    print "done"



