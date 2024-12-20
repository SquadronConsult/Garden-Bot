#!/bin/bash
# Source ROS setup file
source /opt/ros/jazzy/setup.bash

# Source the workspace setup
if [ -f ~/Garden-Bot/install/setup.bash ]; then
    source ~/Garden-Bot/install/setup.bash
fi

# Install GeographicLib datasets
echo "Installing GeographicLib datasets..."
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Install ROS dependencies
echo "Installing ROS dependencies..."
sudo apt update
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-cv-bridge

# Additional environment variables (if any)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS environment configured for Garden-Bot."