# Ensure you have the following installed:

- Docker Desktop: ENSURE YOU HAVE WSL2 ENABLED

- WSL2 INSTALLED

# Then do the following:

1. Run the setup script to configure your environment:
   ```bash
   ./setup.sh
   ```

2. Build the workspace using colcon:
   ```bash
   colcon build
   ```

3. Source the workspace setup script:
   ```bash
   source install/setup.bash
   ```

4. (Optional for testing world not yet created - 12/20/2024) Launch Gazebo with the configured world:
   ```bash
   roslaunch garden_bot gazebo.launch
   ```

5.  Run the MAVLink node to communicate with the quadcopter (If there was a quadcopter to connect):
   ```bash
   rosrun package1 mavlink_node
   ```

6. Set up and run the Docker emulator for Jetson development:
   ```bash
   docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
   docker pull nvcr.io/nvidia/l4t-base:r32.4.3
   docker run --gpus all -it --platform linux/arm64 nvcr.io/nvidia/l4t-base:r32.4.3
   ```
