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
include lib/CMakeFiles/volk_gnsssdr_test_all.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include lib/CMakeFiles/volk_gnsssdr_test_all.dir/compiler_depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/volk_gnsssdr_test_all.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/volk_gnsssdr_test_all.dir/flags.make

lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o: lib/CMakeFiles/volk_gnsssdr_test_all.dir/flags.make
lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o: /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/testqa.cc
lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o: lib/CMakeFiles/volk_gnsssdr_test_all.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o -MF CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o.d -o CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/testqa.cc

lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/testqa.cc > CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.i

lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/testqa.cc -o CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.s

lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o: lib/CMakeFiles/volk_gnsssdr_test_all.dir/flags.make
lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o: /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/qa_utils.cc
lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o: lib/CMakeFiles/volk_gnsssdr_test_all.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o -MF CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o.d -o CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/qa_utils.cc

lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/qa_utils.cc > CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.i

lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib/qa_utils.cc -o CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.s

# Object files for target volk_gnsssdr_test_all
volk_gnsssdr_test_all_OBJECTS = \
"CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o" \
"CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o"

# External object files for target volk_gnsssdr_test_all
volk_gnsssdr_test_all_EXTERNAL_OBJECTS =

lib/volk_gnsssdr_test_all: lib/CMakeFiles/volk_gnsssdr_test_all.dir/testqa.cc.o
lib/volk_gnsssdr_test_all: lib/CMakeFiles/volk_gnsssdr_test_all.dir/qa_utils.cc.o
lib/volk_gnsssdr_test_all: lib/CMakeFiles/volk_gnsssdr_test_all.dir/build.make
lib/volk_gnsssdr_test_all: lib/libvolk_gnsssdr.a
lib/volk_gnsssdr_test_all: lib/CMakeFiles/volk_gnsssdr_test_all.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable volk_gnsssdr_test_all"
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/volk_gnsssdr_test_all.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/volk_gnsssdr_test_all.dir/build: lib/volk_gnsssdr_test_all
.PHONY : lib/CMakeFiles/volk_gnsssdr_test_all.dir/build

lib/CMakeFiles/volk_gnsssdr_test_all.dir/clean:
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/volk_gnsssdr_test_all.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/volk_gnsssdr_test_all.dir/clean

lib/CMakeFiles/volk_gnsssdr_test_all.dir/depend:
	cd /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr /home/juancho/GitHub/gnss-sdr/src/algorithms/libs/volk_gnsssdr_module/volk_gnsssdr/lib /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib /home/juancho/GitHub/gnss-sdr/cmake/volk_gnsssdr_module/build/lib/CMakeFiles/volk_gnsssdr_test_all.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/volk_gnsssdr_test_all.dir/depend

