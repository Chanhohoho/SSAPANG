#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import os, rospkg
import json
import math
import random
from pyzbar.pyzbar import decode

from cv_bridge import CvBridgeError
from sklearn import linear_model

from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class IMGParser:
    def __init__(self, pkg_name = 'ssafy_3'):

        self.img_bgrD = None
        self.img_hsvR = None

        self.image_subD = rospy.Subscriber("/image_jpeg/compressed_D", CompressedImage, self.callbackD)
        self.image_subR = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackR)

        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                self.detectQR()

                rate.sleep()

    def detectQR(self):
        img = cv2.resize(self.img_bgrD, (0,0), fx=0.3, fy=0.3)
        decoded = decode(img)
        print(decoded) 

    def callbackD(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgrD = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
        except CvBridgeError as e:
            print(e)

    def callbackR(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            img_bgrR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_hsvR = cv2.cvtColor(img_bgrR, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)
        
        #self.detectLane("right", img_hsvR)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()