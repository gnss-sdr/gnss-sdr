# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

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

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
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
                ${GNSSSDR_LIB_PATHS}
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
            /opt/homebrew/opt/glog/include/glog
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

# Fix for glog 0.7.0
if(EXISTS ${GLOG_INCLUDE_DIRS}/export.h)
    set_target_properties(Glog::glog PROPERTIES INTERFACE_COMPILE_DEFINITIONS "GLOG_USE_GLOG_EXPORT")
endif()