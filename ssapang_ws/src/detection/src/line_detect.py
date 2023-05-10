#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridgeError

from sensor_msgs.msg import CompressedImage


class IMGParser:
    def __init__(self):

        self.img_bgrD = None

        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                self.binarization()
                self.detectLine()
                rate.sleep()

    def binarization(self):
        lower_blane = np.array([0, 0, 0])
        upper_blane = np.array([50, 50, 50])

        self.img_blane = cv2.inRange(self.img_bgrD, lower_blane, upper_blane)


    def detectLine(self):
        # point1 = self.img_bgrD[240,320]
        # print("p1 == ", point1)

        px = [[0,0], [0,0], [0,0]]
        for i in range(3):
            find = 0
            for j in range(50, 590):
                if find == 0 and self.img_blane[80+ i*80, j]:
                    px[i][0] = j
                    find = 1
                elif find == 1 and self.img_blane[80 + i*80, j] == 0:
                    px[i][1] = j
                    break

        print("\n\npx")
        print(px)


        cv2.line(self.img_bgrD, (px[0][0],80),(px[0][1],80),(0,0,255),5)
        cv2.line(self.img_bgrD, (px[1][0],160),(px[1][1],160),(0,0,255),5)
        cv2.line(self.img_bgrD, (px[2][0],240),(px[2][1],240),(0,0,255),5)
        # for i in range(200, 440):
        #     if self.img_blane[i,180]
        # cv2.line(self.img_blane, (320,240),(320,240),(0,0,255),5)


        cv2.imshow("black", self.img_bgrD)
        cv2.waitKey(1)


    def callbackD(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgrD = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
        self.img_bgrD = cv2.flip(self.img_bgrD, 0)
        # cv2.imshow("bgr", self.img_bgrD)
        # cv2.waitKey(1)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()