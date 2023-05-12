#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import datetime, time
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

global freq
freq = 20

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

def getKey(key):

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

    elif key == 'd':
        # target_linear_vel = checkLinearLimitVelocity(0.1)
        # target_angular_vel = checkAngularLimitVelocity(0)
        # print(vels(target_linear_vel,target_angular_vel))
        # time.sleep(1)
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(-1)
        print(vels(target_linear_vel,target_angular_vel))

    if key == 1:
        target_linear_vel = checkLinearLimitVelocity(0.04)
        target_angular_vel = checkAngularLimitVelocity(1)
        print(vels(target_linear_vel,target_angular_vel))

    elif key == 2:
        target_linear_vel = checkLinearLimitVelocity(0.07)
        target_angular_vel = checkAngularLimitVelocity(0.7)
        print(vels(target_linear_vel,target_angular_vel))

    elif key == 3:
        target_linear_vel = checkLinearLimitVelocity(0.1)
        target_angular_vel = checkAngularLimitVelocity(0)
        print(vels(target_linear_vel,target_angular_vel))

    elif key == 4:
        target_linear_vel = checkLinearLimitVelocity(0.07)
        target_angular_vel = checkAngularLimitVelocity(-0.7)
        print(vels(target_linear_vel,target_angular_vel))

    elif key == 5:
        target_linear_vel = checkLinearLimitVelocity(0.04)
        target_angular_vel = checkAngularLimitVelocity(-1)
        print(vels(target_linear_vel,target_angular_vel))


    elif key == -1:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(-0.8)
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


class IMGParser:
    def __init__(self):

        self.img_bgrD = None

        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
    
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
   
        self.rate = rospy.Rate(freq)

        while not rospy.is_shutdown():
            if self.img_bgrD is not None:
                self.binarization()
                self.detectQR()
                self.rate.sleep()

    def detectQR(self):
        # self.img_bgrD = cv2.resize(self.img_bgrD, (0, 0), fx=1, fy=0.8)
        codes = decode(self.img_bgrD)
        for code in codes:
            qr_info = code.data.decode('utf-8')
            print("\n\n\n\n\n",qr_info)
            qr_ori = code.orientation
            print(qr_ori)

        if codes:
            dir = {'UP':'w', 'DOWN':'a', 'LEFT':'a', 'RIGHT':'d'}

            today = datetime.datetime.today()
            start_time = today.second
            print("\n\n")
            print(start_time)

            for _ in range(3*freq):
                getKey(3)
                self.move_command()
                self.rate.sleep()

            getKey(0)
            self.move_command()
            self.rate.sleep()

            for _ in range(2*freq):
                getKey(dir[qr_ori])
                self.move_command()
                self.rate.sleep()

            getKey(0)
            self.move_command()
            self.rate.sleep()

            for _ in range(3*freq):
                getKey(3)
                self.move_command()
                self.rate.sleep()

            getKey(0)
            self.move_command()
            self.rate.sleep()

        else:
            self.detectLine()
            self.move_command()


    def detectLine(self):
        # point1 = self.img_bgrD[160,320]
        # print("p1 == ", point1)
        # point2 = self.img_bgrD[240,320]
        # print("p1 == ", point2)
        # point3 = self.img_bgrD[320,320]
        # print("p3 == ", point3)
        for i in range(3):
            find = 0
            for j in range(50, 590):
                if find == 0 and self.img_blane[320 - i*80, j]:
                    px[i][0] = j
                    find = 1
                elif find == 1 and self.img_blane[320 - i*80, j] == 0:
                    px[i][1] = j
                    break

        # print("\n\npx")
        # print(px)

        global avg
        denominator = 0
        total = 0
        if 30 < px[0][1]-px[0][0] < 150 and px[0][0] != 50 and px[0][1] != 589:
            cv2.line(self.img_bgrD, (px[0][0],320),(px[0][1],320),(0,0,255),5)
            total += (px[0][1]+px[0][0])/2
            denominator += 1

        if 30 < px[1][1]-px[1][0] < 150 and px[1][0] != 50 and px[1][1] != 589:
            cv2.line(self.img_bgrD, (px[1][0],240),(px[1][1],240),(0,0,255),5)
            total += (px[1][1]+px[1][0])/2
            denominator += 1

        if 30 < px[2][1]-px[2][0] < 150 and px[2][0] != 50 and px[2][1] != 589:
            cv2.line(self.img_bgrD, (px[2][0],160),(px[2][1],160),(0,0,255),5)
            total += (px[2][1]+px[2][0])/2
            denominator += 1
        

        if denominator != 0:
            avg = total/denominator
            if avg < 120:
                print("\n1")
                getKey(1)
            if avg < 220:
                print("\n1")
                getKey(2)
            elif avg < 420:
                print("\n2")
                getKey(3)
            elif avg < 520:
                print("\n2")
                getKey(4)
            elif avg < 640:
                print("\n3")
                getKey(5)
        else:
            getKey(-1)

        # cv2.line(self.img_bgrD, (320,160),(320,160),(0,0,255),5)
        # cv2.line(self.img_bgrD, (320,240),(320,240),(0,0,255),5)
        # cv2.line(self.img_bgrD, (320,320),(320,320),(0,0,255),5)
        # print("avg == ", avg)


        # for i in range(200, 440):
        #     if self.img_blane[i,180]
        # cv2.line(self.img_blane, (320,240),(320,240),(0,0,255),5)

        cv2.imshow("black", self.img_bgrD)
        cv2.waitKey(1)
        # cv2.imshow("img_blane", self.img_blane)
        # cv2.waitKey(1)
    
    def move_command(self):
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
        # cv2.imshow("bgr", self.img_bgrD)
        # cv2.waitKey(1)

if __name__ == '__main__':

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()

    rospy.spin()