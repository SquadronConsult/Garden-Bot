# Ensure you have the following installed:

- Docker Desktop

- Ubuntu 20.04 (Yes, 20.04.. The tool kit is not compatible with 22.04)

- NVIDIA container Toolkit 

# Then do the following:

1. Run the following command to install dependencies:
   ```bash
   ./install_dependencies.sh
   ```

2. Source the ROS2 setup script:
   ```bash
   source ros2_setup.sh
   ```

3. Build the workspace using colcon:
   ```bash
   colcon build
   ```

4. Source the workspace setup script:
   ```bash
   source install/setup.bash
   ```

5. (Optional for testing world not yet created - 12/20/2024) Launch Gazebo with the configured world:
   ```bash
   roslaunch garden_bot gazebo.launch
   ```

6. (Optional for testing world not yet created - 12/20/2024) Run the MAVLink node to communicate with the quadcopter:
   ```bash
   rosrun package1 mavlink_node
   ```

7. Set up and run the Docker emulator for Jetson development:
   ```bash
   ./setup_docker_emulator.sh
   ```
