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
CMAKE_COMMAND = /snap/clion/259/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/259/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/percy/robot_ws/EE3100704

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/percy/robot_ws/EE3100704/cmake-build-debug

# Include any dependencies generated for this target.
include examples/CMakeFiles/canine.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/canine.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/canine.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/canine.dir/flags.make

examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o: examples/canine_autogen/mocs_compilation.cpp
examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o -MF CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o.d -o CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o -c /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/canine_autogen/mocs_compilation.cpp

examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/canine_autogen/mocs_compilation.cpp > CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.i

examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/canine_autogen/mocs_compilation.cpp -o CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.s

examples/CMakeFiles/canine.dir/src/canine.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/src/canine.cpp.o: /home/percy/robot_ws/EE3100704/examples/src/canine.cpp
examples/CMakeFiles/canine.dir/src/canine.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object examples/CMakeFiles/canine.dir/src/canine.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/src/canine.cpp.o -MF CMakeFiles/canine.dir/src/canine.cpp.o.d -o CMakeFiles/canine.dir/src/canine.cpp.o -c /home/percy/robot_ws/EE3100704/examples/src/canine.cpp

examples/CMakeFiles/canine.dir/src/canine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/src/canine.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/examples/src/canine.cpp > CMakeFiles/canine.dir/src/canine.cpp.i

examples/CMakeFiles/canine.dir/src/canine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/src/canine.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/examples/src/canine.cpp -o CMakeFiles/canine.dir/src/canine.cpp.s

examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp
examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o -MF CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o.d -o CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp

examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp > CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.i

examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/cubicTrajectoryGenerator.cpp -o CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.s

examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/forceTrajectoryGenerator.cpp
examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o -MF CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o.d -o CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/forceTrajectoryGenerator.cpp

examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/forceTrajectoryGenerator.cpp > CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.i

examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/forceTrajectoryGenerator.cpp -o CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.s

examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp
examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o -MF CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o.d -o CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp

examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/__/tools/src/robotController.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp > CMakeFiles/canine.dir/__/tools/src/robotController.cpp.i

examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/__/tools/src/robotController.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/robotController.cpp -o CMakeFiles/canine.dir/__/tools/src/robotController.cpp.s

examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp
examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o -MF CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o.d -o CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp

examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/__/tools/src/setTime.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp > CMakeFiles/canine.dir/__/tools/src/setTime.cpp.i

examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/__/tools/src/setTime.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/setTime.cpp -o CMakeFiles/canine.dir/__/tools/src/setTime.cpp.s

examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o: /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp
examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o -MF CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o.d -o CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o -c /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp

examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp > CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.i

examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/tools/src/setObstacle.cpp -o CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.s

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o: /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/mainwindow.cpp
examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o -MF CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o.d -o CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o -c /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/mainwindow.cpp

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/mainwindow.cpp > CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.i

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/mainwindow.cpp -o CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.s

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o: examples/CMakeFiles/canine.dir/flags.make
examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o: /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.cpp
examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o: examples/CMakeFiles/canine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o -MF CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o.d -o CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o -c /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.cpp

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.i"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.cpp > CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.i

examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.s"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/percy/robot_ws/EE3100704/examples/src/robot_UI/robot_ui/qcustomplot.cpp -o CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.s

# Object files for target canine
canine_OBJECTS = \
"CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o" \
"CMakeFiles/canine.dir/src/canine.cpp.o" \
"CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o" \
"CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o" \
"CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o" \
"CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o" \
"CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o" \
"CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o" \
"CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o"

# External object files for target canine
canine_EXTERNAL_OBJECTS =

examples/canine: examples/CMakeFiles/canine.dir/canine_autogen/mocs_compilation.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/src/canine.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/__/tools/src/cubicTrajectoryGenerator.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/__/tools/src/forceTrajectoryGenerator.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/__/tools/src/robotController.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/__/tools/src/setTime.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/__/tools/src/setObstacle.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/mainwindow.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/src/robot_UI/robot_ui/qcustomplot.cpp.o
examples/canine: examples/CMakeFiles/canine.dir/build.make
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.12.8
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.12.8
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.12.8
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
examples/canine: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
examples/canine: examples/CMakeFiles/canine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/percy/robot_ws/EE3100704/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable canine"
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/canine.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/canine.dir/build: examples/canine
.PHONY : examples/CMakeFiles/canine.dir/build

examples/CMakeFiles/canine.dir/clean:
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug/examples && $(CMAKE_COMMAND) -P CMakeFiles/canine.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/canine.dir/clean

examples/CMakeFiles/canine.dir/depend:
	cd /home/percy/robot_ws/EE3100704/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/percy/robot_ws/EE3100704 /home/percy/robot_ws/EE3100704/examples /home/percy/robot_ws/EE3100704/cmake-build-debug /home/percy/robot_ws/EE3100704/cmake-build-debug/examples /home/percy/robot_ws/EE3100704/cmake-build-debug/examples/CMakeFiles/canine.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : examples/CMakeFiles/canine.dir/depend

