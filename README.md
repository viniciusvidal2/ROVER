# ROVER
This repository will contain code for the ROVER developed by GRIN lab.

## Getting started
Clone the repository with its proper submodules in your folder of preference:
```bash
git clone --recursive
cd ROVER
git submodule update --remote
```

Run the following command from inside the ROVER directory to build the docker image in your machine. The image name should be rover_image, with proper tag on it.
```bash
docker build -t rover_image:0.1 .
```

This image should contain the necessary dependencies to use ROS packages and Ardupilot communication through mavlink.

NOTE: Bear in mind the tag you are using when building the image. If for some reason you change it, make sure you do in all the following commands.

## Embedded board setup
First of all, the embedded system expects some sensors and actuators to be connected for correct bootup. The absense of any of those could cause startup to crash. Currently we are using:
- USB camera
- Dynamixel driver and motors
- Livox mid360
Make sure they are properly connected and configured. In the case of livox sensor, make sure the cable network has the proper local network manual IP address (192.168.1.50).

### Installing docker engine on RPI5
The following commands will install the Docker engine with a proper version for raspberry PI 5:

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Building and running the image

We need not only to build the docker image inside the embedded board, but to setup its system to run everything at bootup. To do that, we must create a service with the proper dependencies
- Create a file at /etc/systemd/system/rover_bringup.service with the following command:
```bash
sudo nano /etc/systemd/system/rover_bringup.service
```

- Add the following content to this file:
```
[Unit]
Description=Rover system bringup with docker container and dependencies
After=graphical.target
Wants=graphical.target

[Service]
Type=simple
ExecStart=/home/YOUR_BOARD_USERNAME/ROVER/init_board.sh
Restart=on-failure
RestartSec=10

[Install]
WantedBy=graphical.target
```

- Enable and start the service with the commands:
```bash
sudo systemctl daemon-reload
sudo systemctl enable rover_bringup.service
sudo systemctl start rover_bringup.service
```

You must also enable the system monitor host. Please add the following lines into the ~/.bashrc file:
```
export DISLAY=:0
xhost +
```

To finish up, you must create two folders that will contain the debug bags and the output maps, which will be shared with the docker container:
```
mkdir ~/Desktop/maps
mkdir ~/Desktop/bags_debug
```

You should see the camera prompting in the monitor/radio transmitter next time you bootup. All other sensors should work just fine as well

## Running the container
### Desktop
Use the following command to run the container:
```bash
docker run --restart=always -itd  --network host --name rover_container rover_image:0.1
```

This will not log you inside to container. In case you need to login inside the container from the terminal, use the following command:
```bash
docker exec -it rover_container bash
```

### Embedded (RPI or Jetson boards)
You should not neet to run the container by hand in the embedded board, but you can do it two ways:
- Calling the file that is responsible for starting the entire system at bootup:
```bash
./home/YOUR_BOARD_USERNAME/ROVER/init_board.sh
```

- Running the docker container passing all the necessary info (provided that you have all the sensors connected. If for some reason you don't, remove the config that relates to the missing sensor):
```bash
sudo docker run --restart=always -itd --device=/dev/ttyACM0 --device=/dev/ttyUSB0 --device /dev/video0:/dev/video0 --device /dev/video1:/dev/video1 --privileged --group-add video -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/grin/Desktop/maps:/home/rover/maps -v /home/grin/Desktop/bags_debug:/home/rover/bags_debug --network host --name rover_container rover_image:0.1
```
