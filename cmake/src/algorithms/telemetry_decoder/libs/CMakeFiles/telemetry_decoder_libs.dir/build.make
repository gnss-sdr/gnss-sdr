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
include src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.make

# Include the progress variables for this target.
include src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/progress.make

# Include the compile flags for this target's objects.
include src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o: ../src/algorithms/telemetry_decoder/libs/tlm_conf.cc
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o -MF CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o.d -o CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_conf.cc

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_conf.cc > CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.i

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_conf.cc -o CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.s

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o: ../src/algorithms/telemetry_decoder/libs/tlm_crc_stats.cc
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o -MF CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o.d -o CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_crc_stats.cc

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_crc_stats.cc > CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.i

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_crc_stats.cc -o CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.s

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o: ../src/algorithms/telemetry_decoder/libs/tlm_utils.cc
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o -MF CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o.d -o CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_utils.cc

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_utils.cc > CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.i

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/tlm_utils.cc -o CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.s

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o: ../src/algorithms/telemetry_decoder/libs/viterbi_decoder.cc
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o -MF CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o.d -o CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder.cc

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder.cc > CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.i

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder.cc -o CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.s

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/flags.make
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o: ../src/algorithms/telemetry_decoder/libs/viterbi_decoder_sbas.cc
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o -MF CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o.d -o CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o -c /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder_sbas.cc

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.i"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder_sbas.cc > CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.i

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.s"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs/viterbi_decoder_sbas.cc -o CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.s

# Object files for target telemetry_decoder_libs
telemetry_decoder_libs_OBJECTS = \
"CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o" \
"CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o" \
"CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o" \
"CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o" \
"CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o"

# External object files for target telemetry_decoder_libs
telemetry_decoder_libs_EXTERNAL_OBJECTS =

src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_conf.cc.o
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_crc_stats.cc.o
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/tlm_utils.cc.o
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder.cc.o
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/viterbi_decoder_sbas.cc.o
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/build.make
src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a: src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/juancho/GitHub/gnss-sdr/cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libtelemetry_decoder_libs.a"
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && $(CMAKE_COMMAND) -P CMakeFiles/telemetry_decoder_libs.dir/cmake_clean_target.cmake
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/telemetry_decoder_libs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/build: src/algorithms/telemetry_decoder/libs/libtelemetry_decoder_libs.a
.PHONY : src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/build

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/clean:
	cd /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs && $(CMAKE_COMMAND) -P CMakeFiles/telemetry_decoder_libs.dir/cmake_clean.cmake
.PHONY : src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/clean

src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/depend:
	cd /home/juancho/GitHub/gnss-sdr/cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juancho/GitHub/gnss-sdr /home/juancho/GitHub/gnss-sdr/src/algorithms/telemetry_decoder/libs /home/juancho/GitHub/gnss-sdr/cmake /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs /home/juancho/GitHub/gnss-sdr/cmake/src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/algorithms/telemetry_decoder/libs/CMakeFiles/telemetry_decoder_libs.dir/depend

