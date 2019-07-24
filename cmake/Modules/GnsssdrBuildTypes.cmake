# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.

if(DEFINED __INCLUDED_GNSSSDR_BUILD_TYPES_CMAKE)
    return()
endif()
set(__INCLUDED_GNSSSDR_BUILD_TYPES_CMAKE TRUE)

# Standard CMake Build Types and their basic CFLAGS:
#  - None: nothing set
#  - Debug: -O2 -g
#  - Release: -O3
#  - RelWithDebInfo: -O3 -g
#  - MinSizeRel: -Os

# Additional Build Types, defined below:
#  - NoOptWithASM: -O0 -g -save-temps
#  - O2WithASM: -O2 -g -save-temps
#  - O3WithASM: -O3 -g -save-temps

# Defines the list of acceptable cmake build types. When adding a new
# build type below, make sure to add it to this list.
list(APPEND AVAIL_BUILDTYPES
  None Debug Release RelWithDebInfo MinSizeRel
  Coverage NoOptWithASM O2WithASM O3WithASM ASAN
)

########################################################################
# GNSSSDR_CHECK_BUILD_TYPE(build type)
#
# Use this to check that the build type set in CMAKE_BUILD_TYPE on the
# commandline is one of the valid build types used by this project. It
# checks the value set in the cmake interface against the list of
# known build types in AVAIL_BUILDTYPES. If the build type is found,
# the function exits immediately. If nothing is found by the end of
# checking all available build types, we exit with an error and list
# the available build types.
########################################################################
function(GNSSSDR_CHECK_BUILD_TYPE settype)
  string(TOUPPER ${settype} _settype)
  foreach(btype ${AVAIL_BUILDTYPES})
    string(TOUPPER ${btype} _btype)
    if(${_settype} STREQUAL ${_btype})
      return() # found it; exit cleanly
    endif()
  endforeach()
  # Build type not found; error out
  message(FATAL_ERROR "Build type '${settype}' not valid, must be one of: ${AVAIL_BUILDTYPES}")
endfunction()


########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=Coverage
#
# This type uses no optimization (-O0), outputs debug symbols (-g) and
# outputs all intermediary files the build system produces, including
# all assembly (.s) files. Look in the build directory for these
# files.
# NOTE: This is not defined on Windows systems.
########################################################################
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_COVERAGE "-Wall -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage" CACHE STRING
    "Flags used by the C++ compiler during Coverage builds." FORCE)
  set(CMAKE_C_FLAGS_COVERAGE "-Wall -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage" CACHE STRING
    "Flags used by the C compiler during Coverage builds." FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_COVERAGE
    "-W" CACHE STRING
    "Flags used for linking binaries during Coverage builds." FORCE)
  set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE
    "-W" CACHE STRING
    "Flags used by the shared lib linker during Coverage builds." FORCE)

  mark_as_advanced(
    CMAKE_CXX_FLAGS_COVERAGE
    CMAKE_C_FLAGS_COVERAGE
    CMAKE_EXE_LINKER_FLAGS_COVERAGE
    CMAKE_SHARED_LINKER_FLAGS_COVERAGE)
endif()


########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=NoOptWithASM
#
# This type uses no optimization (-O0), outputs debug symbols (-g) and
# outputs all intermediary files the build system produces, including
# all assembly (.s) files. Look in the build directory for these
# files.
# NOTE: This is not defined on Windows systems.
########################################################################
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_NOOPTWITHASM "-Wall -save-temps -g -O0" CACHE STRING
    "Flags used by the C++ compiler during NoOptWithASM builds." FORCE)
  set(CMAKE_C_FLAGS_NOOPTWITHASM "-Wall -save-temps -g -O0" CACHE STRING
    "Flags used by the C compiler during NoOptWithASM builds." FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_NOOPTWITHASM
    "-W" CACHE STRING
    "Flags used for linking binaries during NoOptWithASM builds." FORCE)
  set(CMAKE_SHARED_LINKER_FLAGS_NOOPTWITHASM
    "-W" CACHE STRING
    "Flags used by the shared lib linker during NoOptWithASM builds." FORCE)

  mark_as_advanced(
    CMAKE_CXX_FLAGS_NOOPTWITHASM
    CMAKE_C_FLAGS_NOOPTWITHASM
    CMAKE_EXE_LINKER_FLAGS_NOOPTWITHASM
    CMAKE_SHARED_LINKER_FLAGS_NOOPTWITHASM)
endif()



########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=O2WithASM
#
# This type uses level 2 optimization (-O2), outputs debug symbols
# (-g) and outputs all intermediary files the build system produces,
# including all assembly (.s) files. Look in the build directory for
# these files.
# NOTE: This is not defined on Windows systems.
########################################################################

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_O2WITHASM "-Wall -save-temps -g -O2" CACHE STRING
    "Flags used by the C++ compiler during O2WithASM builds." FORCE)
  set(CMAKE_C_FLAGS_O2WITHASM "-Wall -save-temps -g -O2" CACHE STRING
    "Flags used by the C compiler during O2WithASM builds." FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_O2WITHASM
    "-W" CACHE STRING
    "Flags used for linking binaries during O2WithASM builds." FORCE)
  set(CMAKE_SHARED_LINKER_FLAGS_O2WITHASM
    "-W" CACHE STRING
    "Flags used by the shared lib linker during O2WithASM builds." FORCE)

  mark_as_advanced(
    CMAKE_CXX_FLAGS_O2WITHASM
    CMAKE_C_FLAGS_O2WITHASM
    CMAKE_EXE_LINKER_FLAGS_O2WITHASM
    CMAKE_SHARED_LINKER_FLAGS_O2WITHASM)
endif()


########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=O3WithASM
#
# This type uses level 3 optimization (-O3), outputs debug symbols
# (-g) and outputs all intermediary files the build system produces,
# including all assembly (.s) files. Look in the build directory for
# these files.
# NOTE: This is not defined on Windows systems.
########################################################################

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_O3WITHASM "-Wall -save-temps -g -O3" CACHE STRING
    "Flags used by the C++ compiler during O3WithASM builds." FORCE)
  set(CMAKE_C_FLAGS_O3WITHASM "-Wall -save-temps -g -O3" CACHE STRING
    "Flags used by the C compiler during O3WithASM builds." FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_O3WITHASM
    "-W" CACHE STRING
    "Flags used for linking binaries during O3WithASM builds." FORCE)
  set(CMAKE_SHARED_LINKER_FLAGS_O3WITHASM
    "-W" CACHE STRING
    "Flags used by the shared lib linker during O3WithASM builds." FORCE)

  mark_as_advanced(
    CMAKE_CXX_FLAGS_O3WITHASM
    CMAKE_C_FLAGS_O3WITHASM
    CMAKE_EXE_LINKER_FLAGS_O3WITHASM
    CMAKE_SHARED_LINKER_FLAGS_O3WITHASM)
endif()


########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=ASAN
#
# This type creates an address sanitized build (-fsanitize=address)
# and defaults to the DebugParanoid linker flags.
# NOTE: This is not defined on Windows systems.
########################################################################
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_ASAN "-Wall -Wextra -g -O2 -fsanitize=address -fno-omit-frame-pointer" CACHE STRING
    "Flags used by the C++ compiler during Address Sanitized builds." FORCE)
  set(CMAKE_C_FLAGS_ASAN "-Wall -Wextra -g -O2 -fsanitize=address -fno-omit-frame-pointer" CACHE STRING
    "Flags used by the C compiler during Address Sanitized builds." FORCE)
  set(CMAKE_EXE_LINKER_FLAGS_ASAN
    "-W" CACHE STRING
    "Flags used for linking binaries during Address Sanitized builds." FORCE)
  set(CMAKE_SHARED_LINKER_FLAGS_ASAN
    "-W" CACHE STRING
    "Flags used by the shared lib linker during Address Sanitized builds." FORCE)

  mark_as_advanced(
    CMAKE_CXX_FLAGS_ASAN
    CMAKE_C_FLAGS_ASAN
    CMAKE_EXE_LINKER_FLAGS_ASAN
    CMAKE_SHARED_LINKER_ASAN)
endif()
