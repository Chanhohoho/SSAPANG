#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from tf.transformations import euler_from_quaternion
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.00

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

def vels(target_linear_vel, target_angular_vel):
  return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def get_odom():
  try:
      (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
      rotation = euler_from_quaternion(rot)

  except (tf.Exception, tf.ConnectivityException, tf.LookupException):
      rospy.loginfo("TF Exception")
      return

  return (Point(*trans), rotation[2])

if __name__=="__main__":
  if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('turtlebot3_teleop')
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

  turtlebot3_model = rospy.get_param("model", "burger")

  status = 0
  target_linear_vel   = 0.0
  target_angular_vel  = 0.0
  control_linear_vel  = 0.0
  control_angular_vel = 0.0
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
    cnt = 0
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      if cnt < 150 :
        control_linear_vel = BURGER_MAX_LIN_VEL
        control_angular_vel = 0.0
      elif cnt <= 155 :
        control_linear_vel = 0.0
        control_angular_vel = 0.0
      elif cnt <= 180:
        control_linear_vel = 0.0
        control_angular_vel = BURGER_MAX_ANG_VEL
      elif cnt <= 185:
        control_linear_vel = 0.0
        control_angular_vel = 0.0
      else:
        cnt = -1
      cnt += 1
      (position, rotation) = get_odom()
      # print(position)
      print(rotation)
      print("\n\n")
      twist = Twist()
      twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
      twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
      # print(cnt, twist)
      pub.publish(twist)
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
