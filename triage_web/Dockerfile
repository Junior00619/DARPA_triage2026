FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Install system deps
RUN apt update && apt install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-tools \
    && rm -rf /var/lib/apt/lists/*

# Python deps
RUN pip3 install flask requests

WORKDIR /app

# Default ROS setup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
