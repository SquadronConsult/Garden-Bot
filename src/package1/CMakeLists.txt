cmake_minimum_required(VERSION 3.8)
project(package1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create an executable for MAVLink node
add_executable(mavlink_node src/mavlink_node.cpp)
target_include_directories(mavlink_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mavlink_node
  rclcpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
)
target_link_libraries(mavlink_node ${dependencies})

# Export dependencies if other packages need them
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
)

# Install the executable
install(TARGETS
  mavlink_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
