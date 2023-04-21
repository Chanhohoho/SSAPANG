#!/usr/bin/env python3

import rospy
import tf
import sys, os
if os.name != 'nt':
  import termios

def get_data():
  print(tf_listener._listener.data)
  

if __name__=="__main__":
  
  if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('tf_pub')
  rate = rospy.Rate(60)
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
        get_data()
        rate.sleep()
  except:
    print('error')

  if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
