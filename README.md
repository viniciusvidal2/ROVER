# ROVER
This repository will contain code for the ROVER developed by GRIN lab.

## Getting started
Clone the repository with its proper submodules in your folder of preference:
```
git clone --recursive
cd ROVER
git submodule update --remote
```

Run the following command from inside the ROVER directory to build the docker image in your machine. The image name should be rover_image, with proper tag on it.
```
docker build -t rover_image:0.1 .
```

This image should contain the necessary dependencies to use ROS packages and Ardupilot communication through mavlink.

## Running the container
Use the following command to run the container:
```
docker run --restart=always -itd  --network host --name rover_container rover_image:0.1
```

This will not log you inside to container. In case you need to login inside the container from the terminal, use the following command:
```
docker exec -it rover_container bash
```
