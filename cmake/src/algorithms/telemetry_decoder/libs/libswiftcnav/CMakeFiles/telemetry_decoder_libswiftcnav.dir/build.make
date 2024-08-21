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
CMAKE_SOURCE_DIR = /home/juancho/GitHub/gnss-sdr

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juancho/GitHub/gnss-sdr/cmake

# Include any dependencies generated for this target.
include src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/compiler_depend.make

# Include the progress variables for this target.
include src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/progress.make

# Include the compile flags for this target's objects.
include src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/flags.make

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/flags.make
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o: ../src/algorithms/telemetry_decoder/libs/libswiftcnav/bits.c
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o -MF CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o.d -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/bits.c

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/bits.c > CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.i

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/bits.c -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.s

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/flags.make
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o: ../src/algorithms/telemetry_decoder/libs/libswiftcnav/cnav_msg.c
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o -MF CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o.d -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/cnav_msg.c

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/cnav_msg.c > CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.i

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/cnav_msg.c -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.s

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/flags.make
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o: ../src/algorithms/telemetry_decoder/libs/libswiftcnav/edc.c
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o -MF CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o.d -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/edc.c

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/edc.c > CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.i

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/edc.c -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.s

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/flags.make
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o: ../src/algorithms/telemetry_decoder/libs/libswiftcnav/viterbi27.c
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o -MF CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o.d -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/viterbi27.c

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/viterbi27.c > CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.i

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav/viterbi27.c -o CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.s

# Object files for target telemetry_decoder_libswiftcnav
telemetry_decoder_libswiftcnav_OBJECTS = \
"CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o" \
"CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o" \
"CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o" \
"CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o"

# External object files for target telemetry_decoder_libswiftcnav
telemetry_decoder_libswiftcnav_EXTERNAL_OBJECTS =

src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/bits.c.o
src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/cnav_msg.c.o
src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/edc.c.o
src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/viterbi27.c.o
src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/build.make
src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a: src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libtelemetry_decoder_libswiftcnav.a"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && $(CMAKE_COMMAND) -P CMakeFiles/telemetry_decoder_libswiftcnav.dir/cmake_clean_target.cmake
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/telemetry_decoder_libswiftcnav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/build: src/algorithms/telemetry_decoder/libs/libswiftcnav/libtelemetry_decoder_libswiftcnav.a
.PHONY : src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/build

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/clean:
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav && $(CMAKE_COMMAND) -P CMakeFiles/telemetry_decoder_libswiftcnav.dir/cmake_clean.cmake
.PHONY : src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/clean

src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/depend:
	cd /home/juancho/GitHub/gnss-sdr/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juancho/GitHub/gnss-sdr /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/libswiftcnav /home/juancho/GitHub/gnss-sdr/cmake /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/algorithms/telemetry_decoder/libs/libswiftcnav/CMakeFiles/telemetry_decoder_libswiftcnav.dir/depend

