# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /snap/clion/260/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/260/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/percy/robot_ws/EE3100704

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/percy/robot_ws/EE3100704/cmake-build-debug

# Include any dependencies generated for this target.
include examples/CMakeFiles/kinova.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/kinova.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/kinova.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/kinova.dir/flags.make

examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o: examples/kinova_autogen/mocs_compilation.cpp
examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o -MF CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o -c /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/kinova_autogen/mocs_compilation.cpp

examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/kinova_autogen/mocs_compilation.cpp > CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.i

examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/kinova_autogen/mocs_compilation.cpp -o CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.s

examples/CMakeFiles/kinova.dir/src/kinova.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/src/kinova.cpp.o: /home/percy/robot_ws/EE3100704/examples/src/kinova.cpp
examples/CMakeFiles/kinova.dir/src/kinova.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/CMakeFiles/kinova.dir/src/kinova.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/src/kinova.cpp.o -MF CMakeFiles/kinova.dir/src/kinova.cpp.o.d -o CMakeFiles/kinova.dir/src/kinova.cpp.o -c /home/percy/robot_ws/EE3100704/examples/src/kinova.cpp

examples/CMakeFiles/kinova.dir/src/kinova.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/src/kinova.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/examples/src/kinova.cpp > CMakeFiles/kinova.dir/src/kinova.cpp.i

examples/CMakeFiles/kinova.dir/src/kinova.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/src/kinova.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/examples/src/kinova.cpp -o CMakeFiles/kinova.dir/src/kinova.cpp.s

examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp
examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o -MF CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o.d -o CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp

examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp > CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i

examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp -o CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s

examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp
examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o -MF CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o.d -o CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp

examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp > CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.i

examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp -o CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.s

examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp
examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o -MF CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o.d -o CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp

examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp > CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.i

examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp -o CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.s

examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o: examples/CMakeFiles/kinova.dir/flags.make
examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp
examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o: examples/CMakeFiles/kinova.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o -MF CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o.d -o CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp

examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp > CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.i

examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp -o CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.s

# Object files for target kinova
kinova_OBJECTS = \
"CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/kinova.dir/src/kinova.cpp.o" \
"CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o" \
"CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o" \
"CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o" \
"CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o"

# External object files for target kinova
kinova_EXTERNAL_OBJECTS =

examples/kinova: examples/CMakeFiles/kinova.dir/kinova_autogen/mocs_compilation.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/src/kinova.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/__/tools/src/robotController.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/__/tools/src/setTime.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/__/tools/src/setObstacle.cpp.o
examples/kinova: examples/CMakeFiles/kinova.dir/build.make
examples/kinova: examples/CMakeFiles/kinova.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable kinova"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinova.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/kinova.dir/build: examples/kinova
.PHONY : examples/CMakeFiles/kinova.dir/build

examples/CMakeFiles/kinova.dir/clean:
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && $(CMAKE_COMMAND) -P CMakeFiles/kinova.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/kinova.dir/clean

examples/CMakeFiles/kinova.dir/depend:
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/percy/robot_ws/EE3100704 /home/percy/robot_ws/EE3100704/examples /home/percy/robot_ws/EE3100704/cmake-build-debug /home/percy/robot_ws/EE3100704/cmake-build-debug/examples /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/CMakeFiles/kinova.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/kinova.dir/depend

