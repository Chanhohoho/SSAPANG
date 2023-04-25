#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import tf
from tf.transformations import euler_from_quaternion
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import sqrt, pow, pi, atan2, degrees
import numpy as np
import random
import sys, os
if os.name != 'nt':
  import termios

arg = rospy.myargv(argv=sys.argv)
robot_name = arg[1]   
now = [float(arg[2]),float(arg[3]),float(arg[4])]
dir =[[-1,0,180],[1,0,0],[0,-1, -90],[0,1, 90]]
nowPosition = Pose()
prv_theta = 0.0
theta_sum = 0.0

def print_pose(msg):
  print(robot_name, "x = %f, y = %f, theta = %f = %f" %(msg.x, msg.y, msg.theta, degrees(msg.theta)))

def get_pose(msg):
  q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
                                      # quart[0] = roll
  quart = euler_from_quaternion(q)    # quart[1] = pitch
  theta = quart[2]                    # quart[2] = yaw <----
  
# make theta within from 0 to 360 degree
  if theta < 0:
      theta = theta + pi * 2
  if theta > pi * 2:
      theta = theta - pi * 2

  pos_x = msg.pose.pose.position.x
  pos_y = msg.pose.pose.position.y

  return pos_x, pos_y, theta

def callback(dat):
  global nowPosition, theta_sum, prv_theta
  pos_x, pos_y, theta = get_pose(dat)
  pose2d       = Pose()   # turtlesim.msg.Pose()
  pose2d.x     = pos_x
  pose2d.y     = pos_y
  pose2d.theta = theta
  
  if   pose2d.theta>  pi: #  5.0(rad) =  286.479(deg)
      pose2d.theta  -= 2 * pi            
  elif pose2d.theta< -pi: # -5.0(rad) = -286.479(deg)
      pose2d.theta  += 2 * pi        

  
  nowPosition = pose2d

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
  rospy.Subscriber('odom', Odometry, callback )

  turtlebot3_model = rospy.get_param("model", "burger")
  move_cmd = Twist()
  rate = rospy.Rate(60)
  target_linear_vel   = 0.0
  target_angular_vel  = 0.0
  
  try:
    while not rospy.is_shutdown():
        linear_speed = 0.22
        angular_speed = 2
        nextIdx()
        print(robot_name , ' - ' , now)
        d = np.deg2rad(now[2])
       
        while True:
            if abs(nowPosition.theta - d) <= 0.02:
               break
            move_cmd.linear.x = 0.00
            if d >= 0:
                if nowPosition.theta <= d and nowPosition.theta >= d - pi:
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.angular.z = -0.5
            else:
                if nowPosition.theta <= d + pi and nowPosition.theta > d:
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.angular.z = 0.5
            pub.publish(move_cmd)
            rate.sleep()

        pub.publish(Twist())
        rate.sleep()


        while True:
            x_start = nowPosition.x
            y_start = nowPosition.y
            path_angle = atan2(now[1] - y_start, now[0]- x_start)

            if path_angle >= 0:
                if nowPosition.theta <= path_angle and nowPosition.theta >= path_angle - pi:
                    move_cmd.angular.z = 0.2
                else:
                    move_cmd.angular.z = -0.2
            else:
                if nowPosition.theta <= path_angle + pi and nowPosition.theta > path_angle:
                    move_cmd.angular.z = -0.2
                else:
                    move_cmd.angular.z = 0.2

            distance = sqrt(pow((now[0] - x_start), 2) + pow((now[1] - y_start), 2))
            if distance <= 0.05:
               break
            move_cmd.linear.x = linear_speed

            pub.publish(move_cmd)
            rate.sleep()
        
        
        rospy.loginfo("Stopping the robot...")
        pub.publish(Twist())
        theta_sum = 0.0
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
