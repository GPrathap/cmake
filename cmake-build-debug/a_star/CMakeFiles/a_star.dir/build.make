# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/geesara/software/clion-2019.1.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/geesara/software/clion-2019.1.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/geesara/project/tmp/cmake

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/geesara/project/tmp/cmake/cmake-build-debug

# Include any dependencies generated for this target.
include a_star/CMakeFiles/a_star.dir/depend.make

# Include the progress variables for this target.
include a_star/CMakeFiles/a_star.dir/progress.make

# Include the compile flags for this target's objects.
include a_star/CMakeFiles/a_star.dir/flags.make

a_star/CMakeFiles/a_star.dir/astar_improved.cpp.o: a_star/CMakeFiles/a_star.dir/flags.make
a_star/CMakeFiles/a_star.dir/astar_improved.cpp.o: ../a_star/astar_improved.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object a_star/CMakeFiles/a_star.dir/astar_improved.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/a_star.dir/astar_improved.cpp.o -c /home/geesara/project/tmp/cmake/a_star/astar_improved.cpp

a_star/CMakeFiles/a_star.dir/astar_improved.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/a_star.dir/astar_improved.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/a_star/astar_improved.cpp > CMakeFiles/a_star.dir/astar_improved.cpp.i

a_star/CMakeFiles/a_star.dir/astar_improved.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/a_star.dir/astar_improved.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/a_star/astar_improved.cpp -o CMakeFiles/a_star.dir/astar_improved.cpp.s

# Object files for target a_star
a_star_OBJECTS = \
"CMakeFiles/a_star.dir/astar_improved.cpp.o"

# External object files for target a_star
a_star_EXTERNAL_OBJECTS =

a_star/liba_star.a: a_star/CMakeFiles/a_star.dir/astar_improved.cpp.o
a_star/liba_star.a: a_star/CMakeFiles/a_star.dir/build.make
a_star/liba_star.a: a_star/CMakeFiles/a_star.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liba_star.a"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && $(CMAKE_COMMAND) -P CMakeFiles/a_star.dir/cmake_clean_target.cmake
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/a_star.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
a_star/CMakeFiles/a_star.dir/build: a_star/liba_star.a

.PHONY : a_star/CMakeFiles/a_star.dir/build

a_star/CMakeFiles/a_star.dir/clean:
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/a_star && $(CMAKE_COMMAND) -P CMakeFiles/a_star.dir/cmake_clean.cmake
.PHONY : a_star/CMakeFiles/a_star.dir/clean

a_star/CMakeFiles/a_star.dir/depend:
	cd /home/geesara/project/tmp/cmake/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/geesara/project/tmp/cmake /home/geesara/project/tmp/cmake/a_star /home/geesara/project/tmp/cmake/cmake-build-debug /home/geesara/project/tmp/cmake/cmake-build-debug/a_star /home/geesara/project/tmp/cmake/cmake-build-debug/a_star/CMakeFiles/a_star.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : a_star/CMakeFiles/a_star.dir/depend

