cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  mavros
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/obstAvoidParams.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obst_detect
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
