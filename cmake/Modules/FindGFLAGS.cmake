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

# - Try to find GFlags
#
# The following variables are optionally searched for defaults
# GFlags_ROOT_DIR: Base directory where all GFlags components are found
#
# The following are set after configuration is done:
# GFlags_FOUND
# GFlags_INCLUDE_DIRS
# GFlags_LIBS
# GFlags_LIBRARY_DIRS

if(APPLE)
    find_path(GFlags_ROOT_DIR
      libgflags.dylib
      PATHS
      /opt/local/lib
      /usr/local/lib
    )
else()
    find_path(GFlags_ROOT_DIR
      libgflags.so
      HINTS
      /usr/local/lib
      /usr/lib/x86_64-linux-gnu
      /usr/lib/i386-linux-gnu
      /usr/lib/arm-linux-gnueabihf
      /usr/lib/arm-linux-gnueabi
      /usr/lib/aarch64-linux-gnu
      /usr/lib/mipsel-linux-gnu
      /usr/lib/mips-linux-gnu
      /usr/lib/mips64el-linux-gnuabi64
      /usr/lib/powerpc-linux-gnu
      /usr/lib/powerpc64-linux-gnu
      /usr/lib/powerpc64le-linux-gnu
      /usr/lib/powerpc-linux-gnuspe
      /usr/lib/hppa-linux-gnu
      /usr/lib/s390x-linux-gnu
      /usr/lib/i386-gnu
      /usr/lib/hppa-linux-gnu
      /usr/lib/x86_64-kfreebsd-gnu
      /usr/lib/i386-kfreebsd-gnu
      /usr/lib/m68k-linux-gnu
      /usr/lib/sh4-linux-gnu
      /usr/lib/sparc64-linux-gnu
      /usr/lib/x86_64-linux-gnux32
      /usr/lib/alpha-linux-gnu
      /usr/lib64
      /usr/lib
    )
endif()

if(GFlags_ROOT_DIR)
    # We are testing only a couple of files in the include directories
    find_path(GFlags_INCLUDE_DIRS
      gflags/gflags.h
      HINTS
      /opt/local/include
      /usr/local/include
      /usr/include
      ${GFlags_ROOT_DIR}/src
    )

    # Find the libraries
    set(GFlags_LIBRARY_DIRS ${GFlags_ROOT_DIR})

    find_library(GFlags_lib gflags ${GFlags_LIBRARY_DIRS})
    if(EXISTS ${GFlags_INCLUDE_DIRS}/gflags/gflags_gflags.h)
      set(GFLAGS_GREATER_20 TRUE)
    else()
      set(GFLAGS_GREATER_20 FALSE)
    endif()
    # set up include and link directory
    include_directories(${GFlags_INCLUDE_DIRS})
    link_directories(${GFlags_LIBRARY_DIRS})
    message(STATUS "gflags library found at ${GFlags_lib}")
    set(GFlags_LIBS ${GFlags_lib})
    set(GFlags_FOUND true)
    mark_as_advanced(GFlags_INCLUDE_DIRS)
else()
    message(STATUS "Cannot find gflags")
    set(GFlags_FOUND false)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFLAGS DEFAULT_MSG GFlags_LIBS GFlags_INCLUDE_DIRS)
