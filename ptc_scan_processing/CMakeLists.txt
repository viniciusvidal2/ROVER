cmake_minimum_required(VERSION 3.0.2)
project(ptc_scan_processing)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    pcl_ros
    tf2_ros
    pcl_conversions
    geometry_msgs
    livox_ros_driver2
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(convert_ptc_laserscan_node src/convert_ptc_laserscan_node.cpp src/cloud_scan_converter.cpp)
target_link_libraries(convert_ptc_laserscan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(convert_ptc_laserscan_node ${catkin_EXPORTED_TARGETS})

add_executable(preprocess_lidar_scan_node src/preprocess_lidar_scan_node.cpp src/scan_preprocessor.cpp)
target_link_libraries(preprocess_lidar_scan_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(preprocess_lidar_scan_node ${catkin_EXPORTED_TARGETS})
