#!/bin/bash
# Source ROS setup file
source /opt/ros/jazzy/setup.bash

# Source the workspace setup
if [ -f ~/Garden-Bot/install/setup.bash ]; then
    source ~/Garden-Bot/install/setup.bash
fi

# Additional environment variables (if any)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "ROS environment configured for Garden-Bot."