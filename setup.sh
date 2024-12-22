#!/bin/bash

set -e

# Check if the image exists
if [[ "$(docker images -q garden-bot:latest 2> /dev/null)" == "" ]]; then
  echo "Building the ROS 2 Docker image..."
  docker build -t garden-bot:latest .
else
  echo "ROS 2 Docker image already exists. Skipping build."
fi

# Pull the DeepStream Docker image
echo "Pulling the DeepStream Docker image..."
docker pull nvcr.io/nvidia/deepstream:7.1-triton-multiarch

# Create a Docker network
echo "Creating a Docker network..."
docker network create garden-net || true

# Run the ROS 2 container
echo "Running the ROS 2 container..."
docker run -d --network garden-net --name garden-bot-container garden-bot:latest

# Run the DeepStream container
echo "Running the DeepStream container..."
docker run -d --network garden-net --name deepstream-container --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,video,graphics \
  nvcr.io/nvidia/deepstream:7.1-triton-multiarch

echo "Setup complete. Containers are running and networked."