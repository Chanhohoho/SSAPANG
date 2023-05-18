#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridgeError
from pyzbar.pyzbar import decode

from sensor_msgs.msg import CompressedImage


def warp_image(img, source_prop):
    
        image_size = (img.shape[1], img.shape[0])

        x = img.shape[1]
        y = img.shape[0]
        
        destination_points = np.float32([
        [0, y],
        [0, 0],
        [x, 0],
        [x, y]
        ])

        source_points = source_prop * np.float32([[x, y]]* 4)

        perspective_transform = cv2.getPerspectiveTransform(source_points, destination_points)

        warped_img = cv2.warpPerspective(img, perspective_transform, image_size, flags=cv2.INTER_LINEAR)
        return warped_img

class IMGParser:
    def __init__(self):

        self.img_bgrD = None
        self.img_hsvR = None

        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
        # self.image_subR = rospy.Subscriber("/image_jpeg/compressed_R", CompressedImage, self.callbackU)

        self.source_prop = np.float32([
                                       [0, 0.9],
                                       [0.37, 0.6],
                                       [0.63, 0.6],
                                       [1, 0.9],
                                       ])

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                # self.img_bgrD = warp_image(self.img_bgrD, self.source_prop)\
                self.binarization()

                self.detectQR()

                rate.sleep()

    # def detectLane(self, camera, img_hsv):
    #     lower_wlane = np.array([0,20,180])
    #     upper_wlane = np.array([45,70,255])

    #     lower_ylane = np.array([15,100,100])
    #     upper_ylane = np.array([30,255,255])
        
    #     img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
    #     img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        
    #     point_w = img_wlane[80,480]
    #     print(point_w)
    #     point_y = img_ylane[80,480]
    #     print(point_y)
    #     if point_w or point_y:
    #         print(camera)
    #     print("\n\n")

    #     cv2.line(img_hsv, (80,480),(80,480),(0,0,255),5)
    #     cv2.imshow(camera, img_hsv)
    #     cv2.waitKey(1)

    def detectQR(self):
        # self.img_bgrD = cv2.resize(self.img_bgrD, (0, 0), fx=1, fy=0.8)
        # decoded = decode(self.img_bgrD)
        # print(decoded)    
        codes = decode(self.img_blane)
        for code in codes:
            qr_info = code.data.decode('utf-8').split(',')[0]
            print("\n\n\n\n\n",qr_info)
            qr_ori = code.orientation
            print(qr_ori)
            
            x, y, w, h = code.rect
            cv2.rectangle(self.img_blane,(x,y),(x+w,y+h), (0, 0, 255), 3)

        cv2.imshow("Image window", self.img_blane)
        cv2.waitKey(1)


    def binarization(self):
        lower_blane = np.array([100,100, 100])
        upper_blane = np.array([255, 255, 255])

        self.img_blane = cv2.inRange(self.img_bgrD, lower_blane, upper_blane)

    def callbackD(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgrD = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # self.img_hsvL = cv2.cvtColor(img_bgrL, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            print(e)
        
        # img_warp = warp_image(self.img_bgrD, self.source_prop)
        # cv2.imshow("Image window", img_warp)
        #self.detectLane("left", img_hsvL)


    # def callbackR(self, msg):
    #     try:
    #         np_arr = np.fromstring(msg.data, np.uint8)
    #         img_bgrR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #         self.img_hsvR = cv2.cvtColor(img_bgrR, cv2.COLOR_BGR2HSV)
    #     except CvBridgeError as e:
    #         print(e)
        
        #self.detectLane("right", img_hsvR)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()