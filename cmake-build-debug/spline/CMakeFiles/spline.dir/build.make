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
include spline/CMakeFiles/spline.dir/depend.make

# Include the progress variables for this target.
include spline/CMakeFiles/spline.dir/progress.make

# Include the compile flags for this target's objects.
include spline/CMakeFiles/spline.dir/flags.make

spline/CMakeFiles/spline.dir/BSpline.cpp.o: spline/CMakeFiles/spline.dir/flags.make
spline/CMakeFiles/spline.dir/BSpline.cpp.o: ../spline/BSpline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object spline/CMakeFiles/spline.dir/BSpline.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline.dir/BSpline.cpp.o -c /home/geesara/project/tmp/cmake/spline/BSpline.cpp

spline/CMakeFiles/spline.dir/BSpline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline.dir/BSpline.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/spline/BSpline.cpp > CMakeFiles/spline.dir/BSpline.cpp.i

spline/CMakeFiles/spline.dir/BSpline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline.dir/BSpline.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/spline/BSpline.cpp -o CMakeFiles/spline.dir/BSpline.cpp.s

spline/CMakeFiles/spline.dir/Bezier.cpp.o: spline/CMakeFiles/spline.dir/flags.make
spline/CMakeFiles/spline.dir/Bezier.cpp.o: ../spline/Bezier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object spline/CMakeFiles/spline.dir/Bezier.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline.dir/Bezier.cpp.o -c /home/geesara/project/tmp/cmake/spline/Bezier.cpp

spline/CMakeFiles/spline.dir/Bezier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline.dir/Bezier.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/spline/Bezier.cpp > CMakeFiles/spline.dir/Bezier.cpp.i

spline/CMakeFiles/spline.dir/Bezier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline.dir/Bezier.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/spline/Bezier.cpp -o CMakeFiles/spline.dir/Bezier.cpp.s

spline/CMakeFiles/spline.dir/CatmullRom.cpp.o: spline/CMakeFiles/spline.dir/flags.make
spline/CMakeFiles/spline.dir/CatmullRom.cpp.o: ../spline/CatmullRom.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object spline/CMakeFiles/spline.dir/CatmullRom.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline.dir/CatmullRom.cpp.o -c /home/geesara/project/tmp/cmake/spline/CatmullRom.cpp

spline/CMakeFiles/spline.dir/CatmullRom.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline.dir/CatmullRom.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/spline/CatmullRom.cpp > CMakeFiles/spline.dir/CatmullRom.cpp.i

spline/CMakeFiles/spline.dir/CatmullRom.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline.dir/CatmullRom.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/spline/CatmullRom.cpp -o CMakeFiles/spline.dir/CatmullRom.cpp.s

spline/CMakeFiles/spline.dir/Curve.cpp.o: spline/CMakeFiles/spline.dir/flags.make
spline/CMakeFiles/spline.dir/Curve.cpp.o: ../spline/Curve.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object spline/CMakeFiles/spline.dir/Curve.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline.dir/Curve.cpp.o -c /home/geesara/project/tmp/cmake/spline/Curve.cpp

spline/CMakeFiles/spline.dir/Curve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline.dir/Curve.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/spline/Curve.cpp > CMakeFiles/spline.dir/Curve.cpp.i

spline/CMakeFiles/spline.dir/Curve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline.dir/Curve.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/spline/Curve.cpp -o CMakeFiles/spline.dir/Curve.cpp.s

spline/CMakeFiles/spline.dir/Vector.cpp.o: spline/CMakeFiles/spline.dir/flags.make
spline/CMakeFiles/spline.dir/Vector.cpp.o: ../spline/Vector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object spline/CMakeFiles/spline.dir/Vector.cpp.o"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spline.dir/Vector.cpp.o -c /home/geesara/project/tmp/cmake/spline/Vector.cpp

spline/CMakeFiles/spline.dir/Vector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spline.dir/Vector.cpp.i"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geesara/project/tmp/cmake/spline/Vector.cpp > CMakeFiles/spline.dir/Vector.cpp.i

spline/CMakeFiles/spline.dir/Vector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spline.dir/Vector.cpp.s"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geesara/project/tmp/cmake/spline/Vector.cpp -o CMakeFiles/spline.dir/Vector.cpp.s

# Object files for target spline
spline_OBJECTS = \
"CMakeFiles/spline.dir/BSpline.cpp.o" \
"CMakeFiles/spline.dir/Bezier.cpp.o" \
"CMakeFiles/spline.dir/CatmullRom.cpp.o" \
"CMakeFiles/spline.dir/Curve.cpp.o" \
"CMakeFiles/spline.dir/Vector.cpp.o"

# External object files for target spline
spline_EXTERNAL_OBJECTS =

spline/libspline.a: spline/CMakeFiles/spline.dir/BSpline.cpp.o
spline/libspline.a: spline/CMakeFiles/spline.dir/Bezier.cpp.o
spline/libspline.a: spline/CMakeFiles/spline.dir/CatmullRom.cpp.o
spline/libspline.a: spline/CMakeFiles/spline.dir/Curve.cpp.o
spline/libspline.a: spline/CMakeFiles/spline.dir/Vector.cpp.o
spline/libspline.a: spline/CMakeFiles/spline.dir/build.make
spline/libspline.a: spline/CMakeFiles/spline.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/geesara/project/tmp/cmake/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libspline.a"
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && $(CMAKE_COMMAND) -P CMakeFiles/spline.dir/cmake_clean_target.cmake
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/spline.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
spline/CMakeFiles/spline.dir/build: spline/libspline.a

.PHONY : spline/CMakeFiles/spline.dir/build

spline/CMakeFiles/spline.dir/clean:
	cd /home/geesara/project/tmp/cmake/cmake-build-debug/spline && $(CMAKE_COMMAND) -P CMakeFiles/spline.dir/cmake_clean.cmake
.PHONY : spline/CMakeFiles/spline.dir/clean

spline/CMakeFiles/spline.dir/depend:
	cd /home/geesara/project/tmp/cmake/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/geesara/project/tmp/cmake /home/geesara/project/tmp/cmake/spline /home/geesara/project/tmp/cmake/cmake-build-debug /home/geesara/project/tmp/cmake/cmake-build-debug/spline /home/geesara/project/tmp/cmake/cmake-build-debug/spline/CMakeFiles/spline.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : spline/CMakeFiles/spline.dir/depend

