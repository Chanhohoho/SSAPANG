#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import time

from geometry_msgs.msg import Twist
import sys, select, os

from cv_bridge import CvBridgeError
from pyzbar.pyzbar import decode

from sensor_msgs.msg import CompressedImage

# BURGER_MAX_LIN_VEL = 0.22
# BURGER_MAX_ANG_VEL = 2.84

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

global status
status = 0
global target_linear_vel
target_linear_vel = 0.0
global target_angular_vel 
target_angular_vel = 0.0
global control_linear_vel
control_linear_vel = 0.0 
global control_angular_vel
control_angular_vel = 0.0

global px
px = [[0,0], [0,0], [0,0]]
global avg
avg = 0

def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


class IMGParser:
    def __init__(self):

        self.img_bgrD = None

        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
    
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
   
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                self.detectQR()
                # self.binarization()
                self.turning()
                rate.sleep()

    def detectQR(self):
        # self.img_bgrD = cv2.resize(self.img_bgrD, (0, 0), fx=1, fy=0.8)
        codes = decode(self.img_bgrD)
        for code in codes:
            qr_info = code.data.decode('utf-8')
            print(qr_info)
            qr_ori = code.orientation
            print(qr_ori)
        if codes:
            dir = {'UP':'w', 'LEFT':'a', 'RIGHT':'d'}
            self.getKey(dir[qr_ori])
        else:
            self.getKey(10)
        self.turning()


    def turning(self):
        global status
        global target_linear_vel
        global target_angular_vel 
        global control_linear_vel
        global control_angular_vel

        twist = Twist()
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        self.pub.publish(twist)

    def getKey(self, key):

        global target_linear_vel
        global target_angular_vel 
        global control_linear_vel
        global control_angular_vel

        if key == 'w':
            target_linear_vel = checkLinearLimitVelocity(0.1)
            target_angular_vel = checkAngularLimitVelocity(0)
            print(vels(target_linear_vel,target_angular_vel))

        elif key == 'a':
            # target_linear_vel = checkLinearLimitVelocity(0.1)
            # target_angular_vel = checkAngularLimitVelocity(0)
            # print(vels(target_linear_vel,target_angular_vel))
            # time.sleep(1)
            target_linear_vel = checkLinearLimitVelocity(0)
            target_angular_vel = checkAngularLimitVelocity(1)
            print(vels(target_linear_vel,target_angular_vel))
            time.sleep(1)

        elif key == 'd':
            # target_linear_vel = checkLinearLimitVelocity(0.1)
            # target_angular_vel = checkAngularLimitVelocity(0)
            # print(vels(target_linear_vel,target_angular_vel))
            # time.sleep(1)
            target_linear_vel = checkLinearLimitVelocity(0)
            target_angular_vel = checkAngularLimitVelocity(-1)
            print(vels(target_linear_vel,target_angular_vel))

        elif key == 10:
            target_linear_vel = checkLinearLimitVelocity(0.1)
            target_angular_vel = checkAngularLimitVelocity(0)
            print(vels(target_linear_vel,target_angular_vel))

        elif key == -1:
            target_linear_vel = checkLinearLimitVelocity(0)
            target_angular_vel = checkAngularLimitVelocity(-1)
            print(vels(target_linear_vel,target_angular_vel))


        elif key == 0:
            target_linear_vel   = 0.0
            control_linear_vel  = 0.0
            target_angular_vel  = 0.0
            control_angular_vel = 0.0
            print(vels(target_linear_vel, target_angular_vel))

        # return key
        
        # for i in range(200, 440):
        #     if self.img_blane[i,180]
        # cv2.line(self.img_blane, (320,240),(320,240),(0,0,255),5)

    def binarization(self):
        lower_blane = np.array([0, 0, 0])
        upper_blane = np.array([100, 100, 100])

        self.img_blane = cv2.inRange(self.img_bgrD, lower_blane, upper_blane)

    def callbackD(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.img_bgrD = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_bgrD = cv2.flip(self.img_bgrD, -1)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("bgr", self.img_bgrD)
        cv2.waitKey(1)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()