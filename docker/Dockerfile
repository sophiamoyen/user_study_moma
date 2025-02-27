# Base image of ubuntu 20.04, cuda 12.1 and ZED SDK 4.1
FROM stereolabs/zed:4.1-devel-cuda12.1-ubuntu20.04

ENV LOGNAME=root
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Setup ROS noetic
RUN apt-get update -y || true && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata curl locales lsb-release gnupg2 mesa-utils apt-transport-https && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    echo 'export LIBGL_ALWAYS_INDIRECT=1' >> ~/.bashrc && \
    apt-get update || true &&\
    DEBIAN_FRONTEND=noninteractive apt-get install -y ros-noetic-desktop-full build-essential cmake usbutils libusb-1.0-0-dev git -y --allow-unauthenticated

# Install Packages
RUN apt-get install -y python3-rosdep python3-rosinstall  python3-rosinstall-generator python3-wstool

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Setup ROS workspace
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws
RUN  . /opt/ros/noetic/setup.sh && \
	    catkin_make 

RUN echo "source /opt/ros_ws/devel/setup.bash" >> ~/.bashrc

# Add user
RUN adduser --quiet --disabled-password qtuser && usermod -a -G audio qtuser

# This fix: libGL error: No matching fbConfigs or visuals found
ENV LIBGL_ALWAYS_INDIRECT=1

# Install Python 3, PyQt5
RUN apt-get update && apt-get install -y python3-pyqt5

RUN bash -c "source /opt/ros_ws/devel/setup.bash"
RUN bash -c "source /opt/ros/noetic/setup.bash"