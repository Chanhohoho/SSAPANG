#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from study_lhj.msg import str

def callback(data):
    print(data)

if __name__ == '__main__':
    rospy.init_node('start_node', anonymous=True)
    pub = rospy.Publisher('/test', str, queue_size=1)
    sub = rospy.Subscriber('/test2', str, callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish('hi')
        rate.sleep()
    rospy.spin()