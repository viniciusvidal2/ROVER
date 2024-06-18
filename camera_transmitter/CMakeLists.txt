cmake_minimum_required(VERSION 2.8.3)
project(camera_transmitter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/camera_transmitter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
