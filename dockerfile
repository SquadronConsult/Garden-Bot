# syntax=docker/dockerfile:1

# Use the official Ubuntu 24.04 (Noble Numbat) base image
FROM ubuntu:24.04

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Enable required repositories and install necessary tools
RUN apt-get install -y software-properties-common curl lsb-release gnupg && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# Install Python 3 and pip
RUN apt-get update && apt-get install -y python3 python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 Jazzy and the specified packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    geographiclib-tools \
    python3-rosdep \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joy* \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros2cli \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Install GeographicLib datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Set environment variables for ROS 2
ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH

# Source ROS 2 setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Copy the entire project into the container
COPY . /root/ros2_ws

# Set the working directory
WORKDIR /root/ros2_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/ros2_ws/install/setup.bash && rm -rf /root/ros2_ws/build && colcon build"

# Set environment variables for display forwarding
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Install necessary packages for display forwarding
RUN apt-get update && apt-get install -y \
    x11-apps \
    libgl1 \
    libxrender1 \
    libxext6 && \
    rm -rf /var/lib/apt/lists/*

# Default command to run when starting the container
CMD ["/bin/bash"]