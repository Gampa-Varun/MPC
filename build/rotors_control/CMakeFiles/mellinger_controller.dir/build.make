# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vgampa/mpc_quad/src/CrazyS/rotors_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vgampa/mpc_quad/build/rotors_control

# Include any dependencies generated for this target.
include CMakeFiles/mellinger_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/mellinger_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/mellinger_controller.dir/flags.make

CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o: CMakeFiles/mellinger_controller.dir/flags.make
CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o: /home/vgampa/mpc_quad/src/CrazyS/rotors_control/src/library/mellinger_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vgampa/mpc_quad/build/rotors_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o -c /home/vgampa/mpc_quad/src/CrazyS/rotors_control/src/library/mellinger_controller.cpp

CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vgampa/mpc_quad/src/CrazyS/rotors_control/src/library/mellinger_controller.cpp > CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.i

CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vgampa/mpc_quad/src/CrazyS/rotors_control/src/library/mellinger_controller.cpp -o CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.s

# Object files for target mellinger_controller
mellinger_controller_OBJECTS = \
"CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o"

# External object files for target mellinger_controller
mellinger_controller_EXTERNAL_OBJECTS =

/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: CMakeFiles/mellinger_controller.dir/src/library/mellinger_controller.cpp.o
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: CMakeFiles/mellinger_controller.dir/build.make
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/libroscpp.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/librosconsole.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/librostime.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /opt/ros/noetic/lib/libcpp_common.so
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so: CMakeFiles/mellinger_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vgampa/mpc_quad/build/rotors_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mellinger_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/mellinger_controller.dir/build: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so

.PHONY : CMakeFiles/mellinger_controller.dir/build

CMakeFiles/mellinger_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mellinger_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mellinger_controller.dir/clean

CMakeFiles/mellinger_controller.dir/depend:
	cd /home/vgampa/mpc_quad/build/rotors_control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vgampa/mpc_quad/src/CrazyS/rotors_control /home/vgampa/mpc_quad/src/CrazyS/rotors_control /home/vgampa/mpc_quad/build/rotors_control /home/vgampa/mpc_quad/build/rotors_control /home/vgampa/mpc_quad/build/rotors_control/CMakeFiles/mellinger_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mellinger_controller.dir/depend

