execute_process(COMMAND "/home/ssafy/S08P31A409/ssapang_ws/build/turtlebot3_autorace_2020/turtlebot3_autorace_driving/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ssafy/S08P31A409/ssapang_ws/build/turtlebot3_autorace_2020/turtlebot3_autorace_driving/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
