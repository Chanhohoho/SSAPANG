roslaunch rrbot_gazebo rrbot_obstacle_world.launch 
roslaunch rrbot_control rrbot_control.launch

rostopic pub -1 /rrbot/joint2_position_controller/command std_msgs/Float64 "data: -1.0"
rostopic pub -1 /rrbot/joint1_position_controller/command std_msgs/Float64 "data: 3.14"
