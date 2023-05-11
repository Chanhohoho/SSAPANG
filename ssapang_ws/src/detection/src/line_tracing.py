#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
import sys, select, os

from cv_bridge import CvBridgeError

from sensor_msgs.msg import CompressedImage

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

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

class IMGParser:
    def __init__(self):

        self.img_bgrD = None

        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
    
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
   
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                self.binarization()
                self.detectLine()
                rate.sleep()

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

def getKey(key):
    if key == 1:
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    elif key == 2:
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    elif key == 3:
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))

    elif key == 0:
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        print(vels(target_linear_vel, target_angular_vel))
    
    return key

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

    if (px[0][0] + px[0][1])/2 < 200:
        getKey(1)
    elif (px[0][0] + px[0][1])/2 < 400:
        getKey(2)
    elif (px[0][0] + px[0][1])/2 < 600:
        getKey(3)

    twist = Twist()
    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    self.pub.publish(twist)

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