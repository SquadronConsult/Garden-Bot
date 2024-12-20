#!/bin/bash
# Update rosdep and install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
