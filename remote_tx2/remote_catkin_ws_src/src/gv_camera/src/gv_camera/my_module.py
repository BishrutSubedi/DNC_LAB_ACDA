
import os

import sys
import rospkg
import rospy
from PyQt5 import uic, QtGui, QtCore
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.uic import *
from PyQt5.QtGui import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from imutils.video import FileVideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2
import Queue
# from dnn_caffe_model import dnn_caffe

  
import threading

class MyPlugin(Plugin):



    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        rp = rospkg.RosPack()

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print 'arguments: ', args
        #     print 'unknowns: ', unknowns

        self.runing = False
        self.capture_thread = None
        # self.q = Queue.Queue()
        # self.i = 4



        # Create QWidget
        self._widget = QWidget()
        v = QVBoxLayout (self._widget)
        self.video_widget =QLabel()
        self.video_widget.setMinimumSize(420,420)
        self.GV_button_start=QPushButton()
        self.GV_button_start.setMaximumSize(160,40)
        self.GV_button_start.setText("START")
        self.GV_button_end = QPushButton()
        self.GV_button_end.setMaximumSize(160, 40)
        self.GV_button_end.setText("END")

        h = QHBoxLayout()
        h.addStretch(1)
        h.addWidget(self.GV_button_start)
        h.addWidget(self.GV_button_end)
        h.addStretch(1)

        v.addWidget(self.video_widget)
        v.addLayout(h)
        self.GV_button_start.clicked.connect(self.start_clicked)
        self.GV_button_end.clicked.connect(self.end_clicked)
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('gv_camera'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # self.capture_thread= threading.Thread(target=self.grap)

        #person detection

        # self.ap = argparse.ArgumentParser()
        # # ap.add_argument("-p", "--prototxt", required=True,
        # # 	help="path to Caffe 'deploy' prototxt file")
        # # ap.add_argument("-m", "--model", required=True,
        # # 	help="path to Caffe pre-trained model")
        # self.ap.add_argument("-c", "--confidence", type=float, default=0.2,
        #                 help="minimum probability to filter weak detections")
        # self.args = vars(self.ap.parse_args())

        # initialize the list of class labels MobileNet SSD was trained to
        # detect, then generate a set of bounding box colors for each class

        #
        # self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        #            "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        #            "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        #            "sofa", "train", "tvmonitor"]

        # CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        # 	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        # 	"dog", "horse", "motorbike", "person", "pottedplant", "sheep"]

        # self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))
        #
        # # load our serialized model from disk


        # print("[INFO] loading model...")
        # self.net = cv2.dnn.readNetFromCaffe("MobileNetSSD_deploy.prototxt.txt", "MobileNetSSD_deploy.caffemodel")
        # model = dnn_caffe
        # self.net = model.net
        # cv2.dnn.rea






    # def update_frame(self):
    #     self.frame = self.fvs.read()
    #     self.frame = imutils.resize(self.frame, width=450)
    #     self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
    #     self.showframe = QtGui.QImage(self.frame.data, self.frame.shape[1], self.shape[0], QtGui.QImage.Format_RGB888)
    #     self.label_show.setPixmap(QtGui.QPixmap.fromImage(self.showframe))
    #     self.fps.update()

    @pyqtSlot()
    def start_clicked(self):
        self.GV_button_start.setEnabled(False)
        self.GV_button_end.setEnabled(True)
        # print("[INFO] starting video stream...")
        # self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))
        #
        # # load our serialized model from disk
        # print("[INFO] loading model...")

        self.capture = cv2.VideoCapture("rtsp://192.168.33.133:8554//CH001.sdp")
        FILE_OUTPUT = 'OUTPUT.avi'
        if os.path.isfile(FILE_OUTPUT):
            os.remove(FILE_OUTPUT)

        # fourcc = cv2.VideoWriter_fourcc(*'MJPEG')
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        self.out = cv2.VideoWriter(FILE_OUTPUT, fourcc, 20.0, (640, 480))
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)
        self.runing = True

        # self.capture_thread.start()



    @pyqtSlot()
    def end_clicked(self):
        self.GV_button_start.setEnabled(True)
        self.GV_button_end.setEnabled(False)
        self.runing = False
        self.timer.stop()
        # self.release()
        self.out.release()
        # self.capture_thread.


    def update_frame(self):
        if (self.capture.isOpened()):
            self.ret, self.frame = self.capture.read()
            if self.ret == True:
                # Handles the mirroring of the current frame
                save_frame = cv2.flip(self.frame, 1)

                # Saves for video
                self.out.write(save_frame)

                # Display the resulting frame
                # cv2.imshow('frame', frame)


            # cv2.imshow("image",self.frame)
            # self.frame = imutils.resize(self.frame, width=400)
            # (h, w) = self.frame.shape[:2]
            # blob = cv2.dnn.blobFromImage(cv2.resize(self.frame, (300, 300)),
            #                              0.007843, (300, 300), 127.5)
            # self.net.setInput(blob)
            # detections = self.net.forward()
            # for i in np.arange(0, detections.shape[2]):
            #     # extract the confidence (i.e., probability) associated with
            #     # the prediction
            #     confidence = detections[0, 0, i, 2]
            #
            #     # filter out weak detections by ensuring the `confidence` is
            #     # greater than the minimum confidence
            #     if confidence > self.args["confidence"]:
            #         # extract the index of the class label from the
            #         # `detections`, then compute the (x, y)-coordinates of
            #         # the bounding box for the object
            #         idx = int(detections[0, 0, i, 1])
            #         box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            #         (startX, startY, endX, endY) = box.astype("int")
            #
            #         # draw the prediction on the frame
            #         label = "{}: {:.2f}%".format(self.CLASSES[idx],
            #                                      confidence * 100)
            #         cv2.rectangle(self.frame, (startX, startY), (endX, endY),
            #                       self.COLORS[idx], 2)
            #         y = startY - 15 if startY - 15 > 15 else startY + 15
            #         cv2.putText(self.frame, label, (startX, y),
            #                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

            image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            heigh, width, channel = image.shape
            step = channel * width
            qimg = QImage(image.data, width, heigh, step, QImage.Format_RGB888)
            self.video_widget.setPixmap(QPixmap.fromImage(qimg))

            # self.video_widget.setText("%f" % self.i)
            # self.video_widget.setPixmap()
            # self.i=1+self.i
            # if not self.q.empty():
            #     frame = self.q.get()
            #     img = frame ["img"]
            #     cv2.imshow("img1", img)
            #     # img = cv2.resize(img,(400,400))
            #
            #     # img = imutils.resize(img, width=400)
            #     img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            #     image = QtGui.QImage(img, img.shape[1], img.shape[0], QtGui.QImage.Format_RGB888 )
            #     self.video_widget.setPixmap(QtGui.QPixmap.fromImage(image))

    def grap(self):
        capture = cv2.VideoCapture("rtsp://192.168.33.133:8554//CH001.sdp")
        while (self.runing):
            _,self.frame = capture.read()
            # print "work"



            # frame = {}
            # capture.grab()
            # retval, img = capture.retrieve(0)
            # # cv2.imshow("img2", img)
            # # print "work2"
            # frame ["img"] = img
            # if self.q.qsize() < 15:
            #     self.q.put(frame)
            # else:
            #     print(self.q.qsize())











    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
