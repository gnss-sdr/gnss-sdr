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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.

# - Try to find the Google Glog library
#
# This module defines the following variables
#
# GLOG_FOUND - Was Glog found
# GLOG_INCLUDE_DIRS - the Glog include directories
# GLOG_LIBRARIES - Link to this
#
# This module accepts the following variables
#
# GLOG_ROOT - Can be set to Glog install path or Windows build path
#

if (NOT DEFINED GLOG_ROOT)
     set (GLOG_ROOT /usr /usr/local)
endif (NOT DEFINED GLOG_ROOT)

if(MSVC)
     set(LIB_PATHS ${GLOG_ROOT} ${GLOG_ROOT}/Release)
else(MSVC)
     set (LIB_PATHS ${GLOG_ROOT} ${GLOG_ROOT}/lib)
endif(MSVC)

macro(_FIND_GLOG_LIBRARIES _var)
     find_library(${_var}
          NAMES  ${ARGN}
          PATHS ${LIB_PATHS} 
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
          PATH_SUFFIXES lib
      )
     mark_as_advanced(${_var})
endmacro()

macro(_GLOG_APPEND_LIBRARIES _list _release)
set(_debug ${_release}_DEBUG)
if(${_debug})
     set(${_list} ${${_list}} optimized ${${_release}} debug ${${_debug}})
else()
     set(${_list} ${${_list}} ${${_release}})
endif()
endmacro()

if(MSVC)
     find_path(GLOG_INCLUDE_DIR NAMES raw_logging.h
          PATHS
          ${GLOG_ROOT}/src/windows
          ${GLOG_ROOT}/src/windows/glog
     )
else(MSVC)
     # Linux/OS X builds
     find_path(GLOG_INCLUDE_DIR NAMES raw_logging.h
     PATHS
     ${GLOG_ROOT}/include/glog
     /usr/include/glog
     /opt/local/include/glog   # default location in Macports
     )
endif(MSVC)

# Find the libraries
if(MSVC)
     _FIND_GLOG_LIBRARIES(GLOG_LIBRARIES libglog.lib)
else(MSVC)
     # Linux/OS X builds
     if(UNIX)
          _FIND_GLOG_LIBRARIES(GLOG_LIBRARIES libglog.so)
     endif(UNIX)
     if(APPLE)
          _FIND_GLOG_LIBRARIES(GLOG_LIBRARIES libglog.dylib)
     endif(APPLE)
endif(MSVC)

if(GLOG_FOUND)
    message(STATUS "glog library found at ${GLOG_LIBRARIES}")
endif()

# handle the QUIETLY and REQUIRED arguments and set GLOG_FOUND to TRUE if
# all listed variables are TRUE
include("${CMAKE_ROOT}/Modules/FindPackageHandleStandardArgs.cmake")
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Glog DEFAULT_MSG
     GLOG_LIBRARIES)

if(MSVC)
     string(REGEX REPLACE "/glog$" "" VAR_WITHOUT ${GLOG_INCLUDE_DIR})
     string(REGEX REPLACE "/windows$" "" VAR_WITHOUT ${VAR_WITHOUT})
     set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIRS} "${VAR_WITHOUT}")
     string(REGEX REPLACE "/libglog.lib" "" GLOG_LIBRARIES_DIR ${GLOG_LIBRARIES})
else(MSVC)
     # Linux/OS X builds
     set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
     string(REGEX REPLACE "/libglog.so" "" GLOG_LIBRARIES_DIR ${GLOG_LIBRARIES})
endif(MSVC)

if(GLOG_FOUND)
      # _GLOG_APPEND_LIBRARIES(GLOG GLOG_LIBRARIES)
endif()
