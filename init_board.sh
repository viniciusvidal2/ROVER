#!/bin/bash

LOGFILE="/home/grin/system_startup.log"

log() {
    echo "$(date +'%Y-%m-%d %H:%M:%S') - $1" | tee -a $LOGFILE
}

# Enable all used USB ports
sleep 5s
echo 1 | sudo -S chmod a+rw /dev/ttyUSB0
echo 1 | sudo -S chmod a+rw /dev/ttyACM0

log "Enabled USB ports permissions."

# Function to check if X server is ready
is_display_ready() {
    if xset q &>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Wait for the display to be ready
while ! is_display_ready; do
    log "Waiting for the display to be ready..."
    sleep 2
done

log "Display service is on!"

# Run the docker container with proper parameters
sleep 5s
echo 1 | sudo -S systemctl enable docker.service
export DOCKER_HOST_NAME=host
export DISPLAY=:0
/usr/bin/docker run --restart=always -itd --device=/dev/ttyACM0 --device=/dev/ttyUSB0 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --privileged --group-add video -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --network $DOCKER_HOST_NAME --name rover_docker rover_image:0.1

log "Docker has been run, system started."