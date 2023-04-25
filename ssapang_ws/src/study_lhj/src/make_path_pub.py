#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
import time
from study_lhj.msg import Locations, Coordinate
import numpy as np

rospy.init_node('make_path_pub', anonymous=True)
pub = rospy.Publisher('/path', Locations, queue_size=1)
rate = rospy.Rate(20)
now = [0,0,0]
dir =[[-0.5,0,180],[0.5,0,0],[0,-0.5, -90],[0,0.5, 90]]
def nextIdx():
    try:
      num = random.randint(0,3)
      for i in range(0,2):
        now[i] += float(dir[num][i])
      now[2] = dir[num][2]
    except:
      print('find next idx error')
sw = True

time.sleep(2)
while not rospy.is_shutdown():
    if sw:
        loc = Locations()
        for i in range(30):
            nextIdx()
            loc.location.append(Coordinate(x=now[0], y=now[1], deg=now[2]))
        pub.publish(loc)
        sw = False
    rate.sleep()