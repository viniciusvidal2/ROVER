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

# Installing catkin tools 
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-pip \
    && pip install future \
    && pip install psutil \
    && pip install dronekit \
    && rm -rf /var/lib/apt/lists/*
RUN catkin config --extend /opt/ros/noetic \
    && catkin init \
    && wstool init src \
    && rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall \
    && rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall \
    && wstool merge -t src /tmp/mavros.rosinstall \
    && wstool update -t src -j4 \
    && rosdep install --from-paths src --ignore-src -y \
    && chmod +x /home/rover/src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
    && ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
    && catkin build || true \
    && catkin build

# Installing mavros and other dependencies
RUN apt-get update \
    && apt-get install -y ros-noetic-mavros \
    && apt-get install -y ros-noetic-mavros-extras \
    && apt-get install -y ros-noetic-tf2-geometry-msgs \
    && apt-get install -y ros-noetic-pcl-ros \
    && apt-get install -y ros-noetic-cv-bridge \
    && apt-get install -y ros-noetic-roslint \
    && apt-get install -y ros-noetic-tf2-eigen \
    && apt-get install -y ros-noetic-tf2-ros \
    && apt-get install -y ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

# Dependencies for camera transmission
RUN apt-get update && apt-get install -y \
    python3-opencv \
    libgtk2.0-dev \
    libgtk-3-dev \
    x11-xserver-utils \
    libgstreamer1.0-0 \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer-plugins-base1.0-dev

# Copy and compile Livox SDK 
COPY Livox-SDK2 /home/rover/src/Livox-SDK2
RUN mkdir -p /home/rover/src/Livox-SDK2/build
WORKDIR /home/rover/src/Livox-SDK2/build
RUN cmake .. && make install

# Copy the packages to inside the docker and compile the ROS ones
WORKDIR /home/rover/
COPY livox_ros_driver2 /home/rover/src/livox_ros_driver2
COPY mig_obstacle_avoidance /home/rover/mig_obstacle_avoidance
COPY camera_transmitter /home/rover/camera_transmitter
COPY livox_filter_mig /home/rover/livox_filter_mig
RUN catkin build

COPY ./start_docker.sh /home/rover/
COPY ./rover_bringup.launch /home/rover/

# Default command to run
CMD [ "bash", "-c", "/home/mig_integration0/start_docker.sh && exec bash" ]
