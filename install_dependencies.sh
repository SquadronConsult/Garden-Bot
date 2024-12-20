#!/bin/bash
# Update rosdep and install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Install Gazebo
sudo apt-get install gazebo11 libgazebo11-dev
