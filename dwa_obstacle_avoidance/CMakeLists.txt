cmake_minimum_required(VERSION 3.0.2)
project(dwa_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mavros
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
