#!/usr/bin/env python3
import rospy
from push_robot_node import push_robot_node
from ssapang.msg import str
import os,sys,time

def pushing(msg):
    burger_node = msg.data
    push_code = push_robot_node.get(burger_node)
    # print(push_code)
    parameter = push_code.split('-')
    # print(parameter[0],parameter[1])
    print(f"rostopic pub -1 /rrbot{parameter[0]}/joint1_position_controller/command std_msgs/Float64 \"data: 1.5\"")
    # os.system('gazebo')
    for i in range (3):
        time.sleep(1)
        print(i+1)
    print(f"rostopic pub -1 /rrbot{parameter[1]}/joint1_position_controller/command std_msgs/Float64 \"data: 1.5\"")

# pushing('BP0303')

if __name__ == '__main__':
    rospy.init_node('pushing')
    sub=rospy.Subscriber('picking', str, pushing)
    rospy.spin()