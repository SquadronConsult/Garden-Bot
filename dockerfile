# Start of Selection
# syntax=docker/dockerfile:1

# Use an official ROS 2 base image
FROM ubuntu:noble

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Enable required repositories
RUN apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy base
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y ros-jazzy-ros-base

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    geographiclib-tools \
    python3-rosdep \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joy* \
    ros-jazzy-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Copy the entire project into the container
COPY . /root/Garden-Bot

# Set the working directory
WORKDIR /root/Garden-Bot

# Clean previous builds
RUN rm -rf build

# Source ROS 2 setup
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Source the workspace setup
RUN echo "source /root/Garden-Bot/install/setup.bash" >> /root/.bashrc

# Default command to run when starting the container
CMD ["/bin/bash"]
# End of Selection