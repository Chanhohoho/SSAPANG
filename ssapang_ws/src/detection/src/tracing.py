#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import datetime, time
from geometry_msgs.msg  import Twist
from ssapang.msg import Move, Locations, Coordinate
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

global ori
ori = 0

global px
px = [[0,0], [0,0], [0,0]]
global avg
avg = 0

global freq
freq = 10

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
    #w 직진
    if key == 0:
        target_linear_vel = checkLinearLimitVelocity(0.05)
        target_angular_vel = checkAngularLimitVelocity(0)
        print(" 직진 ")
        #print(vels(target_linear_vel,target_angular_vel))

    #a 좌회전
    elif key == 270:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(1.2)
        #print(vels(target_linear_vel,target_angular_vel))
        print(" 좌회전 ")

    #d 우회전
    elif key == 90:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(-1.2)
        #print(vels(target_linear_vel,target_angular_vel))
        print(" 우회전 ")

    #s 후진
    if key == 180:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(2)
        #print(vels(target_linear_vel,target_angular_vel))
        print(" 후진 ")


    ##############

    if key == 2:
        target_linear_vel = checkLinearLimitVelocity(-0.05)
        target_angular_vel = checkAngularLimitVelocity(1)
        #print(vels(target_linear_vel,target_angular_vel))

    elif key == 3:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(0.7)
        #print(vels(target_linear_vel,target_angular_vel))

    elif key == 4:
        target_linear_vel = checkLinearLimitVelocity(0.05)
        target_angular_vel = checkAngularLimitVelocity(0.3)
        #print(vels(target_linear_vel,target_angular_vel))

    ##############

    elif key == 5:
        target_linear_vel = checkLinearLimitVelocity(0.08)
        target_angular_vel = checkAngularLimitVelocity(0)
        #print(vels(target_linear_vel,target_angular_vel))

    ##############

    elif key == 6:
        target_linear_vel = checkLinearLimitVelocity(0.05)
        target_angular_vel = checkAngularLimitVelocity(-0.3)
        # print(vels(target_linear_vel,target_angular_vel))

    elif key == 7:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(-0.7)
        # print(vels(target_linear_vel,target_angular_vel))

    elif key == 8:
        target_linear_vel = checkLinearLimitVelocity(-0.05)
        target_angular_vel = checkAngularLimitVelocity(-1)
        # print(vels(target_linear_vel,target_angular_vel))

    elif key == -1:
        target_linear_vel = checkLinearLimitVelocity(0)
        target_angular_vel = checkAngularLimitVelocity(-0.5)
        # print(vels(target_linear_vel,target_angular_vel))
        print("길 찾는중")

    elif key == 1:
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        # print(vels(target_linear_vel, target_angular_vel))

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
        self.image_subD = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callbackD)
    
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.movePub = rospy.Publisher('/move', Move, queue_size=1)
        self.pathSub = rospy.Subscriber("/path",Locations, self.pathCallback)
        self.rate = rospy.Rate(freq)

        self.task = []

        self.img_bgrD = None
        self.img_qr = None
        self.path = []
        self.idx = 0
        self.now = 'B0101'
        self.next = ['B0101', 0]
        self.way_point = 0

        self.inputTask()
        while not rospy.is_shutdown():
            if self.path == []:
                continue
            if self.img_bgrD is not None:
                self.binarization()

                cv2.imshow("qr window", self.img_qr)
                cv2.waitKey(1)

                self.detectQR()
                self.rate.sleep()

    def inputTask(self):
        self.task = input("첫번째 두번째 경로 입력 : ").split()
        self.way_point = 0
        self.makePath()
        
    def makePath(self):
        move = Move()
        move.startNode = self.now
        print(self.way_point)
        move.endNode = self.task[self.way_point]
        self.movePub.publish(move)
        self.rate.sleep()
    
    def nextIdx(self):
        self.now = self.next[0]
        self.idx += 1
        if self.idx >= len(self.path):
            return
        self.next[0] = self.path[self.idx][0]
        self.next[1] = self.path[self.idx-1][1]
        print(self.now, self.next)

    def pathCallback(self, msg):
        for data in msg.location:
            self.path.append([data.QR, data.deg])
        self.idx = 0
        self.next[0] = self.now
        self.nextIdx()
        print(self.next[0])

    def detectQR(self):
        
        # n = input("입력")
        # if n == 'o':
        #     if self.idx+1 == len(self.path):
        #         self.path = []
        #         if self.way_point == 2:
        #             self.inputTask()
        #         else:
        #             self.way_point += 1
        #             self.makePath()
        #     else:
        #         self.nextIdx()
        # else:
        #     self.path = []
        #     self.makePath()

        ###################################################
        # self.img_bgrD = cv2.resize(self.img_bgrD, (0, 0), fx=1, fy=0.8)
        self.img_qrroi = self.img_qr[250:479, 0:639]

        self.qr_info = 0
        self.qr_ori = 0

        codes = decode(self.img_qrroi)
        if len(codes) == 0:
            self.detectLine()
            return
        getKey(1)
        self.move_command()
        self.rate.sleep()
        for code in codes:
            self.qr_info = code.data.decode('utf-8').split(',')[0]
            print("\n\n\n\n\n",self.qr_info)
            self.qr_ori = code.orientation
            print(self.qr_ori)

            x, y, w, h = code.rect
            cv2.rectangle(self.img_blane,(x,y),(x+w,y+h), (0, 0, 255), 3)
        # if qr_info == self.path[-1][0]:
        #     if self.way_point == 3:
        #         self.inputTask()
        #     else:
        #         self.way_point += 1
        #         self.makePath()

        # 같은 qr을 두번 읽었을 때
        if self.qr_info == self.now:
            return
        self.now = self.qr_info

        # 
        if self.qr_info == self.next[0]:
            
            if self.idx+1 == len(self.path):
                self.now = self.path[-2][0]
                print("=====")
                print("self.now=", self.now)
                self.path = []
                self.way_point += 1
                if self.way_point == 2:
                    self.inputTask()
                else:
                    self.makePath()
            else:
                self.nextIdx()
                print("move_hardturn")
                self.move_hardturn()

        # 
        else:
            # self.now = self.path[-2][0]
            self.path = []
            self.makePath()
            
            
            # self.move_command()


        print("self.now == ", self.now)

        print("self.next == ", self.next)

    def move_hardturn(self):

        dir = {'UP':180, 'DOWN':0, 'LEFT':90, 'RIGHT':-90}

        today = datetime.datetime.today()
        start_time = today.second
        # print("\n\n")
        # print(start_time)

        #1초 직진
        print("\nturn_ready")
        getKey(5)
        for _ in range(2*freq):
            self.move_command()
            self.rate.sleep()

        getKey(1)
        self.move_command()
        self.rate.sleep()

        # 각도계산
        print("\nturn")
        # print(self.qr_ori)
        # print(self.next[1])
        move = (dir[self.qr_ori] - self.next[1]) % 360
        # print('move', move)
        getKey(move)
        for _ in range(int(2.5*freq)):
            self.move_command()
            self.rate.sleep()

        getKey(1)
        self.move_command()
        self.rate.sleep()

        #1초 직진
        # for _ in range(1*freq):
        #     getKey(5)
        #     self.move_command()
        #     self.rate.sleep()

        getKey(1)
        self.move_command()
        self.rate.sleep()
        
        #읽고 나서 갱신
        # self.idx += 1

    def detectLine(self):
        global px
        for i in range(3):
            find = 0
            for j in range(50, 590):
                # print(j)
                # print(self.img_blane[320 - i*80, j])
                if find == 0 and self.img_blane[150 - i*50, j]:
                    px[i][0] = j
                    find = 1
                if find == 1 and self.img_blane[150 - i*50, j] == 0:
                    # print("find 1 >> 0")
                    px[i][1] = j
                    break


        # print (px)
        global avg
        denominator = 0
        total = 0
        if 20 < px[0][1]-px[0][0] < 80 and px[0][0] != 50 and px[0][1] != 589:
            # print("l1 == ", px[0][1]-px[0][0])
            cv2.line(self.img_blane, (px[0][0],320),(px[0][1],320),(0,0,255),5)
            total += (px[0][1]+px[0][0])/2
            denominator += 1

        if 20 < px[1][1]-px[1][0] < 70 and px[1][0] != 50 and px[1][1] != 589:
            # print("l2 == ", px[1][1]-px[1][0])
            cv2.line(self.img_blane, (px[1][0],240),(px[1][1],240),(0,0,255),5)
            total += (px[1][1]+px[1][0])/2
            denominator += 1

        if 20 < px[2][1]-px[2][0] < 65 and px[2][0] != 50 and px[2][1] != 589:
            # print("l3 == ", px[2][1]-px[2][0])
            cv2.line(self.img_blane, (px[2][0],160),(px[2][1],160),(0,0,255),5)
            total += (px[2][1]+px[2][0])/2
            denominator += 1
        

        if denominator > 1 :
            avg = total/denominator
            if avg < 120:
                # print("\n2")
                getKey(2)
            elif avg < 190:
                # print("\n3")
                getKey(3)
            elif avg < 260:
                # print("\n4")
                getKey(4)
            elif avg < 380:
                # print("\n5")
                getKey(5)
            elif avg < 450:
                # print("\n6")
                getKey(6)
            elif avg < 520:
                # print("\n7")
                getKey(7)
            elif avg < 590:
                # print("\n8")
                getKey(8)

        else:
            # print("\nfinding")
            getKey(-1)

        self.move_command()
        # cv2.imshow("black", self.img_blane)
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
        upper_blane = np.array([85, 85, 85])
        upper_max = np.array([255, 255, 255])

        self.img_blane = cv2.inRange(self.img_bgrD, lower_blane, upper_blane)
        self.img_qr= cv2.inRange(self.img_bgrD, upper_blane, upper_max)
       

    def callbackD(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.img_bgrD = cv2.flip(img_bgr, -1)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('lane_fitting', anonymous=True)
    image_parser = IMGParser()
    rospy.spin()