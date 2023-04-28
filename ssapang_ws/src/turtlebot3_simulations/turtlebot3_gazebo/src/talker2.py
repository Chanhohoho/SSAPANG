#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi, atan2
import numpy as np
import random
import sys, os
if os.name != 'nt':
  import termios

now = [0.0,0.0,0.0]
dir =[[-1,0,180],[1,0,0],[0,-1, -90],[0,1, 90]]

def get_odom():
  try:
      (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
      rotation = euler_from_quaternion(rot)

  except (tf.Exception, tf.ConnectivityException, tf.LookupException):
      rospy.loginfo("TF Exception")
      return

  return (Point(*trans), rotation[2])

def nextIdx():
    global now
    num = random.randint(0,3)
    for i in range(0,2):
        now[i] += float(dir[num][i])
    now[2] = float(dir[num][2])


if __name__=="__main__":
  if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('turtlebot3_teleop')
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

  turtlebot3_model = rospy.get_param("model", "burger")
  move_cmd = Twist()
  rate = rospy.Rate(60)
  target_linear_vel   = 0.0
  target_angular_vel  = 0.0
  tf_listener = tf.TransformListener()
  odom_frame = 'odom'

  try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
  except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
      tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
      base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
      rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
      rospy.signal_shutdown("tf Exception")

  

  try:
    
    while not rospy.is_shutdown():
        (position, rotation) = get_odom()
        last_rotation = 0
        linear_speed = 0.22
        angular_speed = 2
        nextIdx()
        print(now)
        d = np.deg2rad(now[2])
       
        while True:
            (position, rotation) = get_odom()
            if abs(rotation - d) <= 0.02:
               break
            move_cmd.linear.x = 0.00
            if d >= 0:
                if rotation <= d and rotation >= d - pi:
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= d + pi and rotation > d:
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.angular.z = 0.5
            pub.publish(move_cmd)
            rate.sleep()

        pub.publish(Twist())
        rate.sleep()


        while True:
            (position, rotation) = get_odom() #차량 윛, 방향
            x_start = position.x
            y_start = position.y
            path_angle = atan2(now[1] - y_start, now[0]- x_start)

            if path_angle >= 0:
                if rotation <= path_angle and rotation >= path_angle - pi:
                    move_cmd.angular.z = 0.2
                else:
                    move_cmd.angular.z = -0.2
            else:
                if rotation <= path_angle + pi and rotation > path_angle:
                    move_cmd.angular.z = -0.2
                else:
                    move_cmd.angular.z = 0.2

            distance = sqrt(pow((now[0] - x_start), 2) + pow((now[1] - y_start), 2))
            if distance <= 0.05:
               break
            move_cmd.linear.x = linear_speed

            last_rotation = rotation
            pub.publish(move_cmd)
            rate.sleep()
        
        
        rospy.loginfo("Stopping the robot...")
        pub.publish(Twist())
        rate.sleep()

  except:
    print('error')

  finally:
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)

  if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)