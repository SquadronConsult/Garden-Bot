# syntax=docker/dockerfile:1

# Use the Ubuntu 24.04 base image
FROM ubuntu:24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    ROS_DISTRO=jazzy \
    AMENT_PREFIX_PATH=/opt/ros/jazzy \
    COLCON_PREFIX_PATH=/opt/ros/jazzy \
    LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib \
    PATH=/opt/ros/jazzy/bin:$PATH \
    PYTHONPATH=/opt/ros/jazzy/local/lib/python3.12/dist-packages:/opt/ros/jazzy/lib/python3.12/site-packages \
    ROS_PYTHON_VERSION=3 \
    ROS_VERSION=2 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    tzdata \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN add-apt-repository universe \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

# Install common development tools and libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  openssh-client \
  python3-argcomplete \
  python3-pip \
  ros-dev-tools \
  ros-jazzy-ament-* \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init || echo "rosdep already initialized"

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up autocompletion for user
RUN apt-get update && apt-get install -y git-core bash-completion \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi" >> /home/$USERNAME/.bashrc \
  && rm -rf /var/lib/apt/lists/*

ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# Install the full release
RUN apt-get update && apt-get install -y --no-install-recommends \
  ros-jazzy-desktop \
  && rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH=/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib

# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && apt-get update && apt-get install -y --no-install-recommends \
       ros-jazzy-ros-gz \
       libgz-transport13 \
       && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    nano \
    psmisc \
    qt5ct \
    ros-jazzy-grid-map-rviz-plugin \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-image-geometry \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Install necessary packages for display forwarding
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    libgl1 \
    libxrender1 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

# Install Vulkan and OpenGL libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
    libvulkan1 \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# Install mavros and mavros_msgs dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install xvfb
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Install x11vnc
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Install NoVNC and websockify
RUN apt-get update && apt-get install -y --no-install-recommends \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for display forwarding
ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

# Install NVIDIA Container Toolkit
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | tee /etc/apt/sources.list.d/nvidia-container-toolkit.list \
  && apt-get update \
  && apt-get install -y nvidia-container-toolkit \
  && rm -rf /var/lib/apt/lists/*

# Copy the entire project into the container
COPY . /root/ros2_ws

# Set the working directory
WORKDIR /root/ros2_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Set permissions for setup.bash
RUN chmod +x /root/ros2_ws/install/setup.bash

# Source the workspace setup.bash in .bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Ensure the user sources the workspace
RUN echo "source /root/ros2_ws/install/setup.bash" >> /home/ros/.bashrc

# Default command to run when starting the container
CMD ["/bin/bash"]
