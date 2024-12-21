#!/bin/bash

# Install dependencies
echo "Installing dependencies..."
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install -y gazebo11 libgazebo11-dev

# Source ROS2 setup
echo "Sourcing ROS2 setup..."
source /opt/ros/jazzy/setup.bash
if [ -f ~/Garden-Bot/install/setup.bash ]; then
    source ~/Garden-Bot/install/setup.bash
fi

# Install ROS dependencies
echo "Installing ROS dependencies..."
sudo apt update
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-cv-bridge

# Install GeographicLib tools and datasets
echo "Installing GeographicLib tools and datasets..."
sudo apt update
sudo apt install -y geographiclib-tools
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Install DeepStream SDK dependencies
echo "Installing DeepStream SDK dependencies..."
sudo apt install -y \
libgstreamer1.0-0 \
gstreamer1.0-tools \
gstreamer1.0-plugins-good \
gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly \
gstreamer1.0-libav \
libgstreamer-plugins-base1.0-dev

# Install NVIDIA DeepStream SDK
echo "Installing NVIDIA DeepStream SDK..."
# Remove any previous DeepStream installations
sudo rm -rf /usr/local/deepstream /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnv* /usr/bin/deepstream* /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libnvdsgst* /usr/lib/x86_64-linux-gnu/gstreamer-1.0/deepstream* /opt/nvidia/deepstream/deepstream*
# Download and install the DeepStream 7.1 dGPU Debian package
wget https://catalog.ngc.nvidia.com/orgs/nvidia/resources/deepstream/deepstream-7.1_7.1.0-1_amd64.deb
sudo apt-get install ./deepstream-7.1_7.1.0-1_amd64.deb

echo "Setup complete."