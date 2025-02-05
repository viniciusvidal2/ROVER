# Use the official ROS Noetic base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo
# Set the timezone
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install necessary packages and dependencies
RUN apt-get update && apt-get install -y \
    tzdata curl lsb-release gnupg2 build-essential git nano \
    mosquitto mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# Source ros noetic setup.bash
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Home folder and ROS package path
RUN mkdir -p /home/rover/src
WORKDIR /home/rover

# Environment variables
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV ROS_PACKAGE_PATH=/home/rover/src
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=$ROS_ROOT/lib/python$ROS_PYTHON_VERSION/dist-packages
ENV DISPLAY=:0
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONPATH=/usr/local/lib/python3.8/dist-packages:$PYTHONPATH

# Install apt dependencies for ROS and mavlink
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-pip \
    ros-noetic-mavros ros-noetic-mavros-extras \
    ros-noetic-tf2-geometry-msgs ros-noetic-pcl-ros \
    ros-noetic-cv-bridge ros-noetic-roslint \
    ros-noetic-tf2-eigen ros-noetic-tf2-ros ros-noetic-tf \
    python3-opencv libgtk2.0-dev libgtk-3-dev \
    x11-xserver-utils libgstreamer1.0-0 \
    gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav \
    libgstreamer-plugins-base1.0-dev python3-roslib \
    ros-noetic-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Install pip dependencies
RUN pip install --no-cache-dir future psutil dronekit opencv-python
RUN pip install --no-cache-dir utm datetime scikit-fuzzy networkx flask flask_cors roslibpy paho-mqtt

# Configure Catkin and install MAVROS dependencies
WORKDIR /home/rover/
RUN catkin config --extend /opt/ros/noetic && catkin init \
    && wstool init src \
    && rosinstall_generator --rosdistro noetic mavlink mavros --upstream | tee /tmp/mavros.rosinstall \
    && wstool merge -t src /tmp/mavros.rosinstall \
    && wstool update -t src -j4 \
    && rosdep install --from-paths src --ignore-src -y \
    && catkin build

# Install geographic libs for mavros dependencies
COPY ./install_geographiclib_datasets.sh /home/rover/
RUN chmod +x /home/rover/install_geographiclib_datasets.sh
RUN /home/rover/install_geographiclib_datasets.sh

# Copy and compile Livox SDK 
COPY Livox-SDK2 /home/rover/src/Livox-SDK2
RUN mkdir -p /home/rover/src/Livox-SDK2/build
WORKDIR /home/rover/src/Livox-SDK2/build
RUN cmake .. && make install

# Copy the Dynamixel SDK and install it in the system
COPY DynamixelSDK /home/rover/src/DynamixelSDK
WORKDIR /home/rover/src/DynamixelSDK/python
RUN python3 setup.py install

# Install geographic dependencies
RUN pip install utm datetime
# Install other dependencies
RUN pip install scikit-fuzzy networkx

# Configure Mosquitto
RUN mkdir -p /mosquitto/config/ /mosquitto/data/ /mosquitto/log/
COPY mosquitto.conf /mosquitto/config/mosquitto.conf

# Expose Mosquitto ports
EXPOSE 1883 9001

# Copy the packages to inside the docker and compile the ROS ones
WORKDIR /home/rover/
COPY . /home/rover/src/
# Make sure python scripts are executable
RUN chmod +x /home/rover/src/camera_transmitter/scripts/*.py \
    /home/rover/src/obstacle_avoidance/scripts/*.py \
    /home/rover/src/dwa_obstacle_avoidance/scripts/*.py \
    /home/rover/src/dynamixel_controller/scripts/*.py \
    /home/rover/src/REST_API/scripts/*.py \
    /home/rover/src/pan_and_tilt/*.py
# Build the packages
RUN catkin build -j2

# Source the workspace
RUN echo "source /home/rover/devel/setup.bash" >> /root/.bashrc
RUN /bin/bash -c "source /home/rover/devel/setup.bash"

# Copy the launch files and start script to home folder
COPY ./rover_bringup.launch /home/rover/
COPY ./start_docker.sh /home/rover/
RUN chmod +x /home/rover/start_docker.sh

# Default command to run
CMD [ "bash", "-c", "/home/rover/start_docker.sh && exec bash" ]
