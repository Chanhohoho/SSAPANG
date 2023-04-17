#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from study_lhj.msg import str

def callback(msg):
    print('1', msg.data)
def callback2(msg):
    print('2', msg.data)
rospy.init_node('node2', anonymous=True)
rospy.Subscriber('/test', str, callback)
rospy.Subscriber('/test2', str, callback2)
rospy.spin()