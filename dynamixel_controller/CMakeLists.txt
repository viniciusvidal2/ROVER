cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy mavros 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/dynamixel_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
