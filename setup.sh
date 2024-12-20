#!/bin/bash

# Set up NVIDIA Container Toolkit and Docker for WSL2
echo "Setting up NVIDIA Container Toolkit and Docker for WSL2..."
distribution=ubuntu20.04
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Install dependencies
echo "Installing dependencies..."
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install gazebo11 libgazebo11-dev

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

# Set up Docker emulator for Jetson development
echo "Setting up Docker emulator for Jetson development..."
if ! command -v docker &> /dev/null
then
    echo "Docker could not be found. Please install Docker Desktop."
    exit
fi
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
docker pull nvcr.io/nvidia/l4t-base:r32.4.3
docker run --gpus all -it --platform linux/arm64 nvcr.io/nvidia/l4t-base:r32.4.3

echo "Setup complete."