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
CMAKE_SOURCE_DIR = /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vgampa/mpc_quad/build/rotors_gazebo_plugins

# Include any dependencies generated for this target.
include CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/flags.make

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/flags.make
CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o: /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/gazebo_noisydepth_plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vgampa/mpc_quad/build/rotors_gazebo_plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o -c /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/gazebo_noisydepth_plugin.cpp

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/gazebo_noisydepth_plugin.cpp > CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.i

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/gazebo_noisydepth_plugin.cpp -o CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.s

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/flags.make
CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o: /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/depth_noise_model.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vgampa/mpc_quad/build/rotors_gazebo_plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o -c /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/depth_noise_model.cpp

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/depth_noise_model.cpp > CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.i

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins/src/depth_noise_model.cpp -o CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.s

# Object files for target rotors_gazebo_noisydepth_plugin
rotors_gazebo_noisydepth_plugin_OBJECTS = \
"CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o" \
"CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o"

# External object files for target rotors_gazebo_noisydepth_plugin
rotors_gazebo_noisydepth_plugin_EXTERNAL_OBJECTS =

/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/gazebo_noisydepth_plugin.cpp.o
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/src/depth_noise_model.cpp.o
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/build.make
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: libmav_msgs.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libvision_reconfigure.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_utils.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_camera_utils.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_camera.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_triggered_camera.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_multicamera.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_triggered_multicamera.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_depth_camera.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_openni_kinect.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_gpu_laser.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_laser.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_block_laser.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_p3d.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_imu.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_imu_sensor.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_f3d.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_ft_sensor.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_bumper.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_template.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_projector.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_prosilica.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_force.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_joint_state_publisher.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_joint_pose_trajectory.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_diff_drive.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_tricycle_drive.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_skid_steer_drive.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_video.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_planar_move.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_range.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libgazebo_ros_vacuum_gripper.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libnodeletlib.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libbondcpp.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liburdf.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosconsole_bridge.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libimage_transport.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libcv_bridge.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liboctomap_ros.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liboctomap.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liboctomath.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosbag.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosbag_storage.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libclass_loader.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libroslib.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librospack.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libroslz4.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liblz4.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libtopic_tools.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/liblee_position_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libposition_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libcrazyflie_onboard_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libroll_pitch_yawrate_thrust_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libsensfusion6.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libcrazyflie_complementary_filter.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libmellinger_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libinternal_model_controller.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /home/vgampa/mpc_quad/devel/.private/rotors_control/lib/libroll_pitch_yawrate_thrust_controller_crazyflie.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libtf.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libtf2_ros.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libactionlib.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libroscpp.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libtf2.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosconsole.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/librostime.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/libcpp_common.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.2
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libDepthCameraPlugin.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liboctomap.so.1.9.8
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /opt/ros/noetic/lib/liboctomath.so.1.9.8
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.3.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.6.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.10.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.15.0
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.2
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so: CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vgampa/mpc_quad/build/rotors_gazebo_plugins/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library /home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/build: /home/vgampa/mpc_quad/devel/.private/rotors_gazebo_plugins/lib/librotors_gazebo_noisydepth_plugin.so

.PHONY : CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/build

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/clean

CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/depend:
	cd /home/vgampa/mpc_quad/build/rotors_gazebo_plugins && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins /home/vgampa/mpc_quad/src/CrazyS/rotors_gazebo_plugins /home/vgampa/mpc_quad/build/rotors_gazebo_plugins /home/vgampa/mpc_quad/build/rotors_gazebo_plugins /home/vgampa/mpc_quad/build/rotors_gazebo_plugins/CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rotors_gazebo_noisydepth_plugin.dir/depend

