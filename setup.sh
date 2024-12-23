#!/bin/bash

set -e

MODE=$1

# Function to check if a Docker image exists
check_image_exists() {
  local image_name=$1
  [[ "$(docker images -q $image_name 2> /dev/null)" != "" ]]
}

# Build the ROS 2 Docker image if it doesn't exist
if ! check_image_exists "garden-bot:latest"; then
  echo "Building the ROS 2 Docker image..."
  docker build -t garden-bot:latest .
else
  echo "ROS 2 Docker image already exists. Skipping build."
fi

# Pull the DeepStream Docker image
echo "Pulling the DeepStream Docker image..."
docker pull nvcr.io/nvidia/deepstream:7.1-triton-multiarch

# Ensure no Docker network is running and recreate the network
network_name="garden-net"
if docker network ls | grep -q $network_name; then
  echo "Docker network $network_name already exists. Removing it..."
  docker network rm $network_name
fi
echo "Creating a new Docker network..."
docker network create $network_name

# Run the ROS 2 container with a long-running process
echo "Running the ROS 2 container..."
docker run -d --network $network_name --name garden-bot-container garden-bot:latest /bin/bash -c "while true; do sleep 1000; done"

# Run the DeepStream container with a long-running process
echo "Running the DeepStream container..."
docker run -d --network $network_name --name deepstream-container --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics \
  nvcr.io/nvidia/deepstream:7.1-triton-multiarch /bin/bash -c "while true; do sleep 1000; done"

# Check for missing dependencies in the ROS 2 workspace
echo "Checking for missing dependencies in the ROS 2 workspace..."
docker exec -it garden-bot-container bash -c "sudo rosdep fix-permissions && rosdep update && rosdep check --from-paths /root/ros2_ws/src --ignore-src -r"

# Launch the appropriate mode
case "$MODE" in
  sim)
    echo "Launching Gazebo simulation using ros_gz_sim's launch file in the container..."
    docker exec -it garden-bot-container ros2 launch ros_gz_sim gz_sim.launch.py
    echo "Spawning model in the Gazebo simulation in the container..."
    docker exec -it garden-bot-container ros2 run ros_gz_sim create -world default -file gazebo/models/quadcopter/model.sdf
    ;;
  real)
    echo "Connecting to the real drone..."
    docker exec -it garden-bot-container ros2 launch launch real_drone.launch
    ;;
  *)
    echo "Usage: ./setup.sh [sim|real]"
    exit 1
    ;;
esac

echo "Setup complete. Containers are running and networked."
