#!/bin/bash

set -e

MODE=$1
IMAGE_TAG=${2:-latest}
network_name="garden-net"

# Function to check if a Docker image exists
check_image_exists() {
  local image_name=$1
  [[ "$(docker images -q $image_name 2> /dev/null)" != "" ]]
}

# Check for mode argument
if [ -z "$MODE" ]; then
  echo "Usage: ./setup.sh [sim|real|demo]"
  exit 1
fi

# Build the ROS 2 Docker image if it doesn't exist
if ! check_image_exists "garden-bot:$IMAGE_TAG"; then
  echo "Building the ROS 2 Docker image..."
  docker build -t garden-bot:$IMAGE_TAG .
else
  echo "ROS 2 Docker image already exists. Skipping build."
fi

# Pull the DeepStream Docker image
echo "Pulling the DeepStream Docker image..."
docker pull nvcr.io/nvidia/deepstream:7.1-triton-multiarch

# Ensure no Docker network is running and recreate the network
if docker network ls | grep -q $network_name; then
  echo "Docker network $network_name already exists. Removing it..."
  docker network rm $network_name
fi
echo "Creating a new Docker network..."
docker network create $network_name

# Run the DeepStream container with a long-running process
echo "Running the DeepStream container..."
docker run -d --network $network_name --name deepstream-container --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics \
  nvcr.io/nvidia/deepstream:7.1-triton-multiarch /bin/bash -c "while true; do sleep 1000; done"

# Cleanup existing garden-bot-container if it exists
if docker ps -a | grep -q garden-bot-container; then
  echo "Stopping and removing existing garden-bot-container..."
  docker stop garden-bot-container
  docker rm garden-bot-container
fi

# Run the ROS 2 container with the specified configuration and expose NoVNC port
echo "Running the ROS 2 container..."
docker run -d -p 6080:6080 --network $network_name --name garden-bot-container --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics \
  garden-bot:$IMAGE_TAG /bin/bash -c "while true; do sleep 1000; done"

# Set the DISPLAY environment variable
echo "Setting the DISPLAY environment variable..."
docker exec -d garden-bot-container bash -c "export DISPLAY=:1"
# Start Xvfb after the container is run
echo "Starting Xvfb..."
docker exec -d garden-bot-container bash -c "Xvfb :1 -screen 0 1024x768x16 &"
# Start the VNC server
echo "Starting the VNC server..."
docker exec -d garden-bot-container bash -c "x11vnc -display :1 -forever &"
# Start NoVNC
echo "Starting NoVNC server..."
docker exec -d garden-bot-container bash -c "websockify --web=/usr/share/novnc/ --wrap-mode=ignore 6080 localhost:5900 &"

# Launch the appropriate mode
case "$MODE" in
  sim)
    echo "Launching Gazebo simulation using gazebo_launch.launch.py..."
    docker exec -i garden-bot-container bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch package1 gazebo_launch.launch.py"
    ;;
  real)
    echo "Connecting to the real drone..."
    docker exec -i garden-bot-container bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch package1 real_drone.launch"
    ;;
  demo)
    echo "Launching Gazebo demo using gz_sim.launch.py..."
    docker exec -i garden-bot-container bash -c "source /root/ros2_ws/install/setup.bash && ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf"
    ;;
  *)
    echo "Usage: ./setup.sh [sim|real|demo]"
    exit 1
    ;;
esac

echo "Setup complete. Containers are running and networked."
