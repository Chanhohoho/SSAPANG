#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from study_lhj.msg import Locations, Coordinate

path = []

def callback(msg):
    path = msg.location
    for p in path:
        print(p)
        print('\n\n')

rospy.init_node('node', anonymous=True)
rospy.Subscriber('/path', Locations, callback)
rospy.spin()