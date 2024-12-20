# Ensure you have the following installed:

- Ubuntu 22.04

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

