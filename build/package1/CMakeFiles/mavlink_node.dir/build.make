# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1

# Include any dependencies generated for this target.
include CMakeFiles/mavlink_node.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/mavlink_node.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/mavlink_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mavlink_node.dir/flags.make

CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o: CMakeFiles/mavlink_node.dir/flags.make
CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o: /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1/src/mavlink_node.cpp
CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o: CMakeFiles/mavlink_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o -MF CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o.d -o CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o -c /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1/src/mavlink_node.cpp

CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1/src/mavlink_node.cpp > CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.i

CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1/src/mavlink_node.cpp -o CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.s

# Object files for target mavlink_node
mavlink_node_OBJECTS = \
"CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o"

# External object files for target mavlink_node
mavlink_node_EXTERNAL_OBJECTS =

mavlink_node: CMakeFiles/mavlink_node.dir/src/mavlink_node.cpp.o
mavlink_node: CMakeFiles/mavlink_node.dir/build.make
mavlink_node: /opt/ros/jazzy/lib/librclcpp.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/liblibstatistics_collector.so
mavlink_node: /opt/ros/jazzy/lib/librcl.so
mavlink_node: /opt/ros/jazzy/lib/librmw_implementation.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libtype_description_interfaces__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/librcl_yaml_param_parser.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/librosgraph_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libstatistics_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libtracetools.so
mavlink_node: /opt/ros/jazzy/lib/librcl_logging_interface.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libmavros_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libsensor_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/librcl_interfaces__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeographic_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libgeometry_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libstd_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_c.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_fastrtps_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librmw.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_dynamic_typesupport.so
mavlink_node: /opt/ros/jazzy/lib/libfastcdr.so.2.2.4
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_introspection_c.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_cpp.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_py.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libservice_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libbuiltin_interfaces__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/libunique_identifier_msgs__rosidl_generator_c.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_typesupport_c.so
mavlink_node: /opt/ros/jazzy/lib/librcpputils.so
mavlink_node: /opt/ros/jazzy/lib/librosidl_runtime_c.so
mavlink_node: /opt/ros/jazzy/lib/librcutils.so
mavlink_node: CMakeFiles/mavlink_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mavlink_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mavlink_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mavlink_node.dir/build: mavlink_node
.PHONY : CMakeFiles/mavlink_node.dir/build

CMakeFiles/mavlink_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mavlink_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mavlink_node.dir/clean

CMakeFiles/mavlink_node.dir/depend:
	cd /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1 /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/src/package1 /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1 /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1 /mnt/c/Users/m_jm9/OneDrive/Documents/GitHub/Garden-Bot/build/package1/CMakeFiles/mavlink_node.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/mavlink_node.dir/depend

