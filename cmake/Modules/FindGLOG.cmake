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
#=============================================================================
# FindGlog.cmake, adapted from FindBullet.cmake which has the following
# copyright -
#-----------------------------------------------------------------------------
# Copyright 2009 Kitware, Inc.
# Copyright 2009 Philip Lowman <philip@yhbt.com>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
# License text for the above reference.)

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
NAMES
${ARGN}
PATHS
${LIB_PATHS}
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
)
endif(MSVC)

# Find the libraries
if(MSVC)
_FIND_GLOG_LIBRARIES(GLOG_LIBRARIES libglog.lib)
else(MSVC)
# Linux/OS X builds
_FIND_GLOG_LIBRARIES(GLOG_LIBRARIES libglog.so)
endif(MSVC)

message("glog library = " ${GLOG_LIBRARIES})

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
