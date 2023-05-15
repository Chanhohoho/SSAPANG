#!/usr/bin/env python3
import rospy
from push_robot_node import push_robot_node
from ssapang.msg import str
from std_msgs.msg import Float64
import time

def pushing(msg):

    burger_node = msg.data
    push_code = push_robot_node.get(burger_node)
    parameter = push_code.split('-')
    
    data = Float64()
    data.data = 1.5
    pub[int(parameter[0])].publish(data)

    for i in range (3):
        time.sleep(1)

    data.data = 0
    pub[int(parameter[0])].publish(data)

if __name__ == '__main__':

    rospy.init_node('pushing')

    n = 18
    pub = [None]*(n+1)
    for i in range(1,n+1):
        topic = '/rrbot{num}/joint2_position_controller/command'
        pub[i] = rospy.Publisher(topic.format(num=i), Float64, queue_size=1)

    sub=rospy.Subscriber('picking', str, pushing)

    rospy.spin()