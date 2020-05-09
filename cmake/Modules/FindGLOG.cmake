# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

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
# Provides the following imported target:
# Glog::glog
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT DEFINED GLOG_ROOT)
    set(GLOG_ROOT /usr /usr/local)
endif()

if(MSVC)
    set(LIB_PATHS ${GLOG_ROOT} ${GLOG_ROOT}/Release)
else()
    set(LIB_PATHS ${GLOG_ROOT} ${GLOG_ROOT}/lib)
endif()

pkg_check_modules(PC_GLOG libglog)

macro(_FIND_GLOG_LIBRARIES _var)
    find_library(${_var}
          NAMES ${ARGN}
          HINTS ${PC_GLOG_LIBDIR}
          PATHS ${LIB_PATHS}
                /usr/lib
                /usr/lib64
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
                /usr/lib/riscv64-linux-gnu
                /usr/local/lib
                /usr/local/lib64
                /opt/local/lib
                ${GLOG_ROOT}/lib
                $ENV{GLOG_ROOT}/lib
                ${GLOG_ROOT}/lib64
                $ENV{GLOG_ROOT}/lib64
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
        HINTS
            ${PC_GLOG_INCLUDEDIR}
        PATHS
            ${GLOG_ROOT}/src/windows
            ${GLOG_ROOT}/src/windows/glog
    )
else()
    # Linux/OS X builds
    find_path(GLOG_INCLUDE_DIR NAMES raw_logging.h
        HINTS
            ${PC_GLOG_INCLUDEDIR}
        PATHS
            /usr/include/glog
            /usr/local/include/glog
            /opt/local/include/glog   # default location in Macports
            ${GLOG_ROOT}/include/glog
    )
endif()

# Find the libraries
if(MSVC)
    _find_glog_libraries(GLOG_LIBRARIES libglog.lib)
else()
    # Linux/OS X builds
    if(UNIX)
        _find_glog_libraries(GLOG_LIBRARIES libglog.so)
    endif()
    if(APPLE)
        _find_glog_libraries(GLOG_LIBRARIES glog)
    endif()
endif()

# handle the QUIETLY and REQUIRED arguments and set GLOG_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLOG DEFAULT_MSG GLOG_LIBRARIES)

if(GLOG_FOUND)
    message(STATUS "glog library found at ${GLOG_LIBRARIES}")
    if(PC_GLOG_VERSION)
        set(GLOG_VERSION ${PC_GLOG_VERSION})
    endif()
endif()

if(MSVC)
    string(REGEX REPLACE "/glog$" "" VAR_WITHOUT ${GLOG_INCLUDE_DIR})
    string(REGEX REPLACE "/windows$" "" VAR_WITHOUT ${VAR_WITHOUT})
    set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIRS} "${VAR_WITHOUT}")
    string(REGEX REPLACE "/libglog.lib" "" GLOG_LIBRARIES_DIR ${GLOG_LIBRARIES})
else()
    # Linux/OS X builds
    set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
    string(REGEX REPLACE "/libglog.so" "" GLOG_LIBRARIES_DIR ${GLOG_LIBRARIES})
endif()

if(GLOG_FOUND AND GLOG_VERSION)
    set_package_properties(GLOG PROPERTIES
        DESCRIPTION "C++ implementation of the Google logging module (found: v${GLOG_VERSION})"
    )
else()
    set_package_properties(GLOG PROPERTIES
        DESCRIPTION "C++ implementation of the Google logging module"
    )
endif()

set_package_properties(GLOG PROPERTIES
    URL "https://github.com/google/glog"
)

string(REGEX MATCH libglog.a GLOG_IS_STATIC ${GLOG_LIBRARIES})
if(GLOG_FOUND AND NOT TARGET Glog::glog)
    if(GLOG_IS_STATIC)
        add_library(Glog::glog STATIC IMPORTED)
    else()
        add_library(Glog::glog SHARED IMPORTED)
    endif()
    set_target_properties(Glog::glog PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GLOG_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${GLOG_LIBRARIES}"
    )
endif()
