#!/bin/bash

# Install GeographicLib tools and datasets
echo "Installing GeographicLib tools and datasets..."
sudo apt update
sudo apt install -y geographiclib-tools
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Install ROS dependencies
echo "Installing ROS dependencies..."
sudo apt update
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-cv-bridge

# Install python3-rosdep just in case
echo "Installing python3-rosdep just in case..."
sudo apt-get install -y python3-rosdep

# Source the ROS 2 installation
echo "Sourcing the ROS 2 installation..."
source /opt/ros/jazzy/setup.bash

# Source the workspace overlay if it exists
if [ -f install/setup.bash ]; then
    echo "Sourcing the workspace overlay..."
    source install/setup.bash
fi

# Add any additional environment setup here

# Set ROS_DISTRO environment variable
echo "Setting ROS_DISTRO environment variable..."
echo "export ROS_DISTRO=jazzy" >> ~/.bashrc
source ~/.bashrc

# Set up NVIDIA Container Toolkit
echo "Setting up NVIDIA Container Toolkit..."
distribution=ubuntu20.04
curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' | \
sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

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

# Set up DeepStream
echo "Setting up NVIDIA DeepStream..."
sudo docker run -it --rm --gpus all --network=host \
-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
nvcr.io/nvidia/deepstream:7.1-triton-multiarch

echo "Setup complete."