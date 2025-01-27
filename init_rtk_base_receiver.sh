#!/bin/bash

# Enable all used USB ports
sleep 5s
echo 1 | sudo -S chmod a+rw /dev/ttyUSB0
echo 1 | sudo -S chmod a+rw /dev/ttyACM0

log "Enabled USB ports permissions."

# Run the RTK base receiver python script
sleep 25s
echo 1 | sudo -S python3 /home/mig/ROVER/server_functions/rtk_communication/base_survey_mavlink_udp_robots.py
