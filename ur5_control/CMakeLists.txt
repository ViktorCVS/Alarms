cmake_minimum_required(VERSION 3.0.2)
project(ur5_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  trajectory_msgs
  gazebo_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/ur5_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

