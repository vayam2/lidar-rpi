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
include lightwarelidar/CMakeFiles/platformLinux.dir/depend.make

# Include the progress variables for this target.
include lightwarelidar/CMakeFiles/platformLinux.dir/progress.make

# Include the compile flags for this target's objects.
include lightwarelidar/CMakeFiles/platformLinux.dir/flags.make

lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o: lightwarelidar/CMakeFiles/platformLinux.dir/flags.make
lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o: /home/indowings/sf_45/src/lightwarelidar/src/linux/platformLinux.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/indowings/sf_45/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o -c /home/indowings/sf_45/src/lightwarelidar/src/linux/platformLinux.cpp

lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.i"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/indowings/sf_45/src/lightwarelidar/src/linux/platformLinux.cpp > CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.i

lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.s"
	cd /home/indowings/sf_45/build/lightwarelidar && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/indowings/sf_45/src/lightwarelidar/src/linux/platformLinux.cpp -o CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.s

# Object files for target platformLinux
platformLinux_OBJECTS = \
"CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o"

# External object files for target platformLinux
platformLinux_EXTERNAL_OBJECTS =

/home/indowings/sf_45/devel/lib/libplatformLinux.so: lightwarelidar/CMakeFiles/platformLinux.dir/src/linux/platformLinux.cpp.o
/home/indowings/sf_45/devel/lib/libplatformLinux.so: lightwarelidar/CMakeFiles/platformLinux.dir/build.make
/home/indowings/sf_45/devel/lib/libplatformLinux.so: lightwarelidar/CMakeFiles/platformLinux.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/indowings/sf_45/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/indowings/sf_45/devel/lib/libplatformLinux.so"
	cd /home/indowings/sf_45/build/lightwarelidar && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/platformLinux.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lightwarelidar/CMakeFiles/platformLinux.dir/build: /home/indowings/sf_45/devel/lib/libplatformLinux.so

.PHONY : lightwarelidar/CMakeFiles/platformLinux.dir/build

lightwarelidar/CMakeFiles/platformLinux.dir/clean:
	cd /home/indowings/sf_45/build/lightwarelidar && $(CMAKE_COMMAND) -P CMakeFiles/platformLinux.dir/cmake_clean.cmake
.PHONY : lightwarelidar/CMakeFiles/platformLinux.dir/clean

lightwarelidar/CMakeFiles/platformLinux.dir/depend:
	cd /home/indowings/sf_45/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/indowings/sf_45/src /home/indowings/sf_45/src/lightwarelidar /home/indowings/sf_45/build /home/indowings/sf_45/build/lightwarelidar /home/indowings/sf_45/build/lightwarelidar/CMakeFiles/platformLinux.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lightwarelidar/CMakeFiles/platformLinux.dir/depend

