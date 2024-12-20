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

# Source additional ROS2 setup
echo "Sourcing additional ROS2 setup..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Setup complete."