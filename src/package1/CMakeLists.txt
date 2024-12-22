cmake_minimum_required(VERSION 3.10)
project(package1)

# Set the C++ standard to C++17 and enforce it
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

# Add compiler warnings for GCC, Clang, and MSVC
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create an executable for MAVLink node
add_executable(mavlink_node src/mavlink_node.cpp)
target_include_directories(mavlink_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mavlink_node
  rclcpp
  sensor_msgs
  mavros_msgs
)

# Install the executable
install(TARGETS
  mavlink_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
