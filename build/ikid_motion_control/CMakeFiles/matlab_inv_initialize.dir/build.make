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
CMAKE_SOURCE_DIR = /home/wp/ikid_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wp/ikid_ws/build

# Include any dependencies generated for this target.
include ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/depend.make

# Include the progress variables for this target.
include ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/progress.make

# Include the compile flags for this target's objects.
include ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/flags.make

ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o: ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/flags.make
ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o: /home/wp/ikid_ws/src/ikid_motion_control/src/source/matlab_inv_initialize.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wp/ikid_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o"
	cd /home/wp/ikid_ws/build/ikid_motion_control && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o   -c /home/wp/ikid_ws/src/ikid_motion_control/src/source/matlab_inv_initialize.c

ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.i"
	cd /home/wp/ikid_ws/build/ikid_motion_control && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/wp/ikid_ws/src/ikid_motion_control/src/source/matlab_inv_initialize.c > CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.i

ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.s"
	cd /home/wp/ikid_ws/build/ikid_motion_control && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/wp/ikid_ws/src/ikid_motion_control/src/source/matlab_inv_initialize.c -o CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.s

# Object files for target matlab_inv_initialize
matlab_inv_initialize_OBJECTS = \
"CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o"

# External object files for target matlab_inv_initialize
matlab_inv_initialize_EXTERNAL_OBJECTS =

/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/src/source/matlab_inv_initialize.c.o
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/build.make
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/libroscpp.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/librosconsole.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/librostime.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /opt/ros/noetic/lib/libcpp_common.so
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so: ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wp/ikid_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library /home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so"
	cd /home/wp/ikid_ws/build/ikid_motion_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/matlab_inv_initialize.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/build: /home/wp/ikid_ws/devel/lib/libmatlab_inv_initialize.so

.PHONY : ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/build

ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/clean:
	cd /home/wp/ikid_ws/build/ikid_motion_control && $(CMAKE_COMMAND) -P CMakeFiles/matlab_inv_initialize.dir/cmake_clean.cmake
.PHONY : ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/clean

ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/depend:
	cd /home/wp/ikid_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wp/ikid_ws/src /home/wp/ikid_ws/src/ikid_motion_control /home/wp/ikid_ws/build /home/wp/ikid_ws/build/ikid_motion_control /home/wp/ikid_ws/build/ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ikid_motion_control/CMakeFiles/matlab_inv_initialize.dir/depend

