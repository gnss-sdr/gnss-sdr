# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build

# Include any dependencies generated for this target.
include apps/CMakeFiles/volk_gnsssdr-config-info.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include apps/CMakeFiles/volk_gnsssdr-config-info.dir/compiler_depend.make

# Include the progress variables for this target.
include apps/CMakeFiles/volk_gnsssdr-config-info.dir/progress.make

# Include the compile flags for this target's objects.
include apps/CMakeFiles/volk_gnsssdr-config-info.dir/flags.make

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o: apps/CMakeFiles/volk_gnsssdr-config-info.dir/flags.make
apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o: /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr-config-info.cc
apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o: apps/CMakeFiles/volk_gnsssdr-config-info.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o -MF CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o.d -o CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr-config-info.cc

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr-config-info.cc > CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.i

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr-config-info.cc -o CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.s

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o: apps/CMakeFiles/volk_gnsssdr-config-info.dir/flags.make
apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o: /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr_option_helpers.cc
apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o: apps/CMakeFiles/volk_gnsssdr-config-info.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o -MF CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o.d -o CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr_option_helpers.cc

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr_option_helpers.cc > CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.i

apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps/volk_gnsssdr_option_helpers.cc -o CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.s

# Object files for target volk_gnsssdr-config-info
volk_gnsssdr__config__info_OBJECTS = \
"CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o" \
"CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o"

# External object files for target volk_gnsssdr-config-info
volk_gnsssdr__config__info_EXTERNAL_OBJECTS =

apps/volk_gnsssdr-config-info: apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr-config-info.cc.o
apps/volk_gnsssdr-config-info: apps/CMakeFiles/volk_gnsssdr-config-info.dir/volk_gnsssdr_option_helpers.cc.o
apps/volk_gnsssdr-config-info: apps/CMakeFiles/volk_gnsssdr-config-info.dir/build.make
apps/volk_gnsssdr-config-info: lib/libvolk_gnsssdr.a
apps/volk_gnsssdr-config-info: apps/CMakeFiles/volk_gnsssdr-config-info.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable volk_gnsssdr-config-info"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/volk_gnsssdr-config-info.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/CMakeFiles/volk_gnsssdr-config-info.dir/build: apps/volk_gnsssdr-config-info
.PHONY : apps/CMakeFiles/volk_gnsssdr-config-info.dir/build

apps/CMakeFiles/volk_gnsssdr-config-info.dir/clean:
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps && $(CMAKE_COMMAND) -P CMakeFiles/volk_gnsssdr-config-info.dir/cmake_clean.cmake
.PHONY : apps/CMakeFiles/volk_gnsssdr-config-info.dir/clean

apps/CMakeFiles/volk_gnsssdr-config-info.dir/depend:
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/apps /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/apps/CMakeFiles/volk_gnsssdr-config-info.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : apps/CMakeFiles/volk_gnsssdr-config-info.dir/depend

