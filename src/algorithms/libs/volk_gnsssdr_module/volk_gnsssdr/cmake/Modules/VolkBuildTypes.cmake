# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

if(DEFINED __INCLUDED_VOLK_BUILD_TYPES_CMAKE)
    return()
endif()
set(__INCLUDED_VOLK_BUILD_TYPES_CMAKE TRUE)

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
#  - DebugParanoid -O0 -g -Werror

# Defines the list of acceptable cmake build types. When adding a new
# build type below, make sure to add it to this list.
list(APPEND AVAIL_BUILDTYPES
  None Debug Release RelWithDebInfo MinSizeRel
  DebugParanoid Coverage NoOptWithASM O2WithASM O3WithASM
  ASAN
)

########################################################################
# VOLK_CHECK_BUILD_TYPE(build type)
#
# Use this to check that the build type set in CMAKE_BUILD_TYPE on the
# commandline is one of the valid build types used by this project. It
# checks the value set in the cmake interface against the list of
# known build types in AVAIL_BUILDTYPES. If the build type is found,
# the function exits immediately. If nothing is found by the end of
# checking all available build types, we exit with an error and list
# the available build types.
########################################################################
function(VOLK_CHECK_BUILD_TYPE settype)
  string(TOUPPER ${settype} _settype)
  foreach(btype ${AVAIL_BUILDTYPES})
    string(TOUPPER ${btype} _btype)
    if(${_settype} STREQUAL ${_btype})
      return() # found it; exit cleanly
    endif()
  endforeach()
  # Build type not found; warn out at set it to None
  message(STATUS "Warning: Build type '${settype}' not valid, must be one of: ${AVAIL_BUILDTYPES}.")
  message(STATUS "Setting the build type to 'None'")
  set(CMAKE_BUILD_TYPE "None" PARENT_SCOPE)
endfunction()

########################################################################
# For GCC and Clang, we can set a build type:
#
# -DCMAKE_BUILD_TYPE=DebugParanoid
#
# This type uses no optimization (-O0), outputs debug symbols (-g), warns
# on everything, and stops on warnings.
# NOTE: This is not defined on Windows systems.
########################################################################
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS_DEBUGPARANOID "-Wall -Wextra -g -O0" CACHE STRING
    "Flags used by the C++ compiler during DebugParanoid builds." FORCE)
  set(CMAKE_C_FLAGS_DEBUGPARANOID "-Wall -Wextra -g -O0" CACHE STRING
    "Flags used by the C compiler during DebugParanoid builds." FORCE)
  if(CMAKE_COMPILER_ID STREQUAL GNU)
    set(CMAKE_EXE_LINKER_FLAGS_DEBUGPARANOID
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used for linking binaries during DebugParanoid builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUGPARANOID
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used by the shared lib linker during DebugParanoid builds." FORCE)
  else()
    set(CMAKE_EXE_LINKER_FLAGS_DEBUGPARANOID
      "-Wl" CACHE STRING
      "Flags used for linking binaries during DebugParanoid builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_DEBUGPARANOID
      "-Wl" CACHE STRING
      "Flags used by the shared lib linker during DebugParanoid builds." FORCE)
  endif()

  mark_as_advanced(
    CMAKE_CXX_FLAGS_DEBUGPARANOID
    CMAKE_C_FLAGS_DEBUGPARANOID
    CMAKE_EXE_LINKER_FLAGS_DEBUGPARANOID
    CMAKE_SHARED_LINKER_DEBUGPARANOID)
endif()

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
  set(CMAKE_CXX_FLAGS_NOOPTWITHASM "-save-temps -g -O0" CACHE STRING
    "Flags used by the C++ compiler during NoOptWithASM builds." FORCE)
  set(CMAKE_C_FLAGS_NOOPTWITHASM "-save-temps -g -O0" CACHE STRING
    "Flags used by the C compiler during NoOptWithASM builds." FORCE)
  if(CMAKE_COMPILER_ID STREQUAL GNU)
    set(CMAKE_EXE_LINKER_FLAGS_NOOPTWITHASM
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used for linking binaries during NoOptWithASM builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_NOOPTWITHASM
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used by the shared lib linker during NoOptWithASM builds." FORCE)
  else()
    set(CMAKE_EXE_LINKER_FLAGS_NOOPTWITHASM
      "-Wl" CACHE STRING
      "Flags used for linking binaries during NoOptWithASM builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_NOOPTWITHASM
      "-Wl" CACHE STRING
      "Flags used by the shared lib linker during NoOptWithASM builds." FORCE)
  endif()
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
  set(CMAKE_CXX_FLAGS_O2WITHASM "-save-temps -g -O2" CACHE STRING
    "Flags used by the C++ compiler during O2WithASM builds." FORCE)
  set(CMAKE_C_FLAGS_O2WITHASM "-save-temps -g -O2" CACHE STRING
    "Flags used by the C compiler during O2WithASM builds." FORCE)
  if(CMAKE_COMPILER_ID STREQUAL GNU)
    set(CMAKE_EXE_LINKER_FLAGS_O2WITHASM
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used for linking binaries during O2WithASM builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_O2WITHASM
      "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
      "Flags used by the shared lib linker during O2WithASM builds." FORCE)
  else()
    set(CMAKE_EXE_LINKER_FLAGS_O2WITHASM
      "-Wl" CACHE STRING
      "Flags used for linking binaries during O2WithASM builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_O2WITHASM
      "-Wl" CACHE STRING
      "Flags used by the shared lib linker during O2WithASM builds." FORCE)
  endif()

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
  set(CMAKE_CXX_FLAGS_O3WITHASM "-save-temps -g -O3" CACHE STRING
    "Flags used by the C++ compiler during O3WithASM builds." FORCE)
  set(CMAKE_C_FLAGS_O3WITHASM "-save-temps -g -O3" CACHE STRING
    "Flags used by the C compiler during O3WithASM builds." FORCE)
    if(CMAKE_COMPILER_ID STREQUAL GNU)
      set(CMAKE_EXE_LINKER_FLAGS_O3WITHASM
        "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
        "Flags used for linking binaries during O3WithASM builds." FORCE)
      set(CMAKE_SHARED_LINKER_FLAGS_O3WITHASM
        "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
        "Flags used by the shared lib linker during O3WithASM builds." FORCE)
    else()
      set(CMAKE_EXE_LINKER_FLAGS_O3WITHASM
        "-Wl" CACHE STRING
        "Flags used for linking binaries during O3WithASM builds." FORCE)
      set(CMAKE_SHARED_LINKER_FLAGS_O3WITHASM
        "-Wl" CACHE STRING
        "Flags used by the shared lib linker during O3WithASM builds." FORCE)
  endif()

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
  mark_as_advanced(
    CMAKE_CXX_FLAGS_ASAN
    CMAKE_C_FLAGS_ASAN
    CMAKE_EXE_LINKER_FLAGS_DEBUGPARANOID
    CMAKE_SHARED_LINKER_DEBUGPARANOID)
endif()
