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
CMAKE_SOURCE_DIR = /home/indowings/sf_45/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/indowings/sf_45/build

# Include any dependencies generated for this target.
include lightwarelidar/CMakeFiles/sf40c.dir/depend.make

# Include the progress variables for this target.
include lightwarelidar/CMakeFiles/sf40c.dir/progress.make

# Include the compile flags for this target's objects.
include lightwarelidar/CMakeFiles/sf40c.dir/flags.make

lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.o: lightwarelidar/CMakeFiles/sf40c.dir/flags.make
lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.o: /home/indowings/sf_45/src/lightwarelidar/src/sf40c.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/indowings/sf_45/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.o"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sf40c.dir/src/sf40c.cpp.o -c /home/indowings/sf_45/src/lightwarelidar/src/sf40c.cpp

lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sf40c.dir/src/sf40c.cpp.i"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/indowings/sf_45/src/lightwarelidar/src/sf40c.cpp > CMakeFiles/sf40c.dir/src/sf40c.cpp.i

lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sf40c.dir/src/sf40c.cpp.s"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/indowings/sf_45/src/lightwarelidar/src/sf40c.cpp -o CMakeFiles/sf40c.dir/src/sf40c.cpp.s

# Object files for target sf40c
sf40c_OBJECTS = \
"CMakeFiles/sf40c.dir/src/sf40c.cpp.o"

# External object files for target sf40c
sf40c_EXTERNAL_OBJECTS =

/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: lightwarelidar/CMakeFiles/sf40c.dir/src/sf40c.cpp.o
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: lightwarelidar/CMakeFiles/sf40c.dir/build.make
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /home/indowings/sf_45/devel/lib/liblwNx.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /home/indowings/sf_45/devel/lib/libplatformLinux.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /home/indowings/sf_45/devel/lib/libserialPortLinux.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/libroscpp.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/librosconsole.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/librostime.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /opt/ros/noetic/lib/libcpp_common.so
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/indowings/sf_45/devel/lib/lightwarelidar/sf40c: lightwarelidar/CMakeFiles/sf40c.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/indowings/sf_45/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/indowings/sf_45/devel/lib/lightwarelidar/sf40c"
	cd /home/indowings/sf_45/build/lightwarelidar && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sf40c.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lightwarelidar/CMakeFiles/sf40c.dir/build: /home/indowings/sf_45/devel/lib/lightwarelidar/sf40c

.PHONY : lightwarelidar/CMakeFiles/sf40c.dir/build

lightwarelidar/CMakeFiles/sf40c.dir/clean:
	cd /home/indowings/sf_45/build/lightwarelidar && $(CMAKE_COMMAND) -P CMakeFiles/sf40c.dir/cmake_clean.cmake
.PHONY : lightwarelidar/CMakeFiles/sf40c.dir/clean

lightwarelidar/CMakeFiles/sf40c.dir/depend:
	cd /home/indowings/sf_45/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/indowings/sf_45/src /home/indowings/sf_45/src/lightwarelidar /home/indowings/sf_45/build /home/indowings/sf_45/build/lightwarelidar /home/indowings/sf_45/build/lightwarelidar/CMakeFiles/sf40c.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lightwarelidar/CMakeFiles/sf40c.dir/depend

