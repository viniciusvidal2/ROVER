#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/rover/devel/setup.bash
nohup roslaunch /home/rover/rover_bringup.launch &
echo "ROS environment is running"
