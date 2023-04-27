#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import sqrt, pow, pi, atan2
from study_lhj.msg import Locations, Coordinate
import numpy as np
import random
import sys, os
if os.name != 'nt':
  import termios

class robot:
  def __init__(self):
    rospy.init_node('robot')
    arg = rospy.myargv(argv=sys.argv)
    self.robot_name = arg[1]   
    self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('odom', Odometry, self.callback )
    rospy.Subscriber('/path', Locations, self.path_callback)

    self.move_cmd = Twist()
    self.rate = rospy.Rate(60)

    self.idx = 0
    self.path = []
    self.now = [0,0,0]
    self.nowPosition = Pose()
    self.linear_speed = 0.22

    try:
      while not rospy.is_shutdown():
        if len(self.path) > 0:
          self.nextIdx()
          print(self.robot_name , ' - ' , self.now)
          self.turn()
          self.go()        
          
          print("arrival idx...")
          self.idx += 1
    finally:
      twist = Twist()
      twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
      self.cmd_pub.publish(twist)

  def path_callback(self, msg):
    self.path = msg.location

  def callback(self, msg):
    pose2d = Pose()   # turtlesim.msg.Pose()
    pose2d.x, pose2d.y , pose2d.theta = self.get_pose(msg)
    
    if pose2d.theta >  pi: #  3.14(rad) =  180(deg)
      pose2d.theta -= 2 * pi            
    elif pose2d.theta < -pi: # -3.14(rad) = -180(deg)
      pose2d.theta += 2 * pi        
    print(pose2d.theta)
    print()
    self.nowPosition = pose2d

  def get_pose(self, msg):
    q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                                        # quart[0] = roll
    quart = euler_from_quaternion(q)    # quart[1] = pitch
    theta = quart[2]                    # quart[2] = yaw <----
    
  # make theta within from 0 to 360 degree
    print(theta)
    # if theta < 0:
    #     theta += pi * 2
    # elif theta > pi * 2:
    #     theta -= pi * 2
    # print(theta)

    return msg.pose.pose.position.x, msg.pose.pose.position.y, theta

  def nextIdx(self):
    try:
      self.now[0] = self.path[self.idx].x
      self.now[1] = self.path[self.idx].y
      self.now[2] = self.path[self.idx].deg
      self.d = np.deg2rad(self.now[2])
    except:
      print('find next idx error')

  def turn(self):
    try:
      self.move_cmd.linear.x = 0.00
      while True:
        if abs(self.nowPosition.theta - self.d) <= 0.02:
          return
        if self.d >= 0:
          if self.nowPosition.theta <= self.d and self.nowPosition.theta >= self.d - pi:
            self.move_cmd.angular.z = 0.5
          else:
            self.move_cmd.angular.z = -0.5
        else:
          if self.nowPosition.theta <= self.d + pi and self.nowPosition.theta > self.d:
            self.move_cmd.angular.z = -0.5
          else:
            self.move_cmd.angular.z = 0.5
        self.cmd_pub.publish(self.move_cmd)
        self.rate.sleep()
    except:
      print('turn error')
    finally:
      self.cmd_pub.publish(Twist())
      self.rate.sleep()

  def go(self):
    try:
      while True:
        x_start = self.nowPosition.x
        y_start = self.nowPosition.y
        path_angle = atan2(self.now[1] - y_start, self.now[0]- x_start)

        if path_angle >= 0:
          if self.nowPosition.theta <= path_angle and self.nowPosition.theta >= path_angle - pi:
            self.move_cmd.angular.z = 0.2
          else:
            self.move_cmd.angular.z = -0.2
        else:
          if self.nowPosition.theta <= path_angle + pi and self.nowPosition.theta > path_angle:
            self.move_cmd.angular.z = -0.2
          else:
            self.move_cmd.angular.z = 0.2

        distance = sqrt(pow((self.now[0] - x_start), 2) + pow((self.now[1] - y_start), 2))
        if distance <= 0.05:
          return
        self.move_cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(self.move_cmd)
        self.rate.sleep()
    except:
      print('go error')
    finally:
      self.cmd_pub.publish(Twist())
      self.rate.sleep()


if __name__=="__main__":
  if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
  robot()
  
  if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
