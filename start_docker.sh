#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/rover/devel/setup.bash
nohup roslaunch /home/rover/rover_bringup.launch &
echo "ROS environment is running"

# Directory to save the bag files
SAVE_DIR="/home/rover/bags_debug"
# Topics to record
TOPICS="/livox/scan /mavros/imu/data /mavros/setpoint_raw/target_global /mavros/state /mavros/global_position/global /mavros/global_position/compass_hdg /mavros/mission/waypoints /mavros/home_position/home"
# Duration to record each bag file (in seconds)
DURATION=60
# Construct the filename
FILENAME="$SAVE_DIR/rosbag_debug"
# Record the topics
nohup rosbag record $TOPICS -o $FILENAME --split --duration=$DURATION &
