#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from study_lhj.msg import str

def callback(data):
    print(data)

rospy.init_node('node', anonymous=True)
sub = rospy.Subscriber('/test', str, callback)
pub = rospy.Publisher('/test2', str, queue_size=1)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    pub.publish('bye')
    rate.sleep()
rospy.spin()