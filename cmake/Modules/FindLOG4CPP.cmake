# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

# - Find Log4cpp
# Find the native LOG4CPP includes and library
#
#  LOG4CPP_INCLUDE_DIR - where to find LOG4CPP.h, etc.
#  LOG4CPP_LIBRARIES   - List of libraries when using LOG4CPP.
#  LOG4CPP_FOUND       - True if LOG4CPP found.
#
# Provides the following imported target:
# Log4cpp::log4cpp
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

pkg_check_modules(PC_LOG4CPP log4cpp QUIET)

if(LOG4CPP_INCLUDE_DIR)
  # Already in cache, be silent
  set(LOG4CPP_FIND_QUIETLY TRUE)
endif()

if(LOG4CPP_ROOT)
    set(LOG4CPP_ROOT_USER_PROVIDED ${LOG4CPP_ROOT})
else()
    set(LOG4CPP_ROOT_USER_PROVIDED /usr)
endif()
if(DEFINED ENV{LOG4CPP_ROOT})
    set(LOG4CPP_ROOT_USER_PROVIDED
        ${LOG4CPP_ROOT_USER_PROVIDED}
        $ENV{LOG4CPP_ROOT}
    )
endif()
set(LOG4CPP_ROOT_USER_PROVIDED
    ${LOG4CPP_ROOT_USER_PROVIDED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(LOG4CPP_INCLUDE_DIR log4cpp/Category.hh
    ${LOG4CPP_ROOT_USER_PROVIDED}/include
    /usr/include
    /usr/local/include
    /opt/local/include
    ${PC_LOG4CPP_INCLUDEDIR}
)

if(LOG4CPP_INCLUDE_DIR)
    file(STRINGS ${LOG4CPP_INCLUDE_DIR}/log4cpp/Priority.hh _log4cpp_Priority)
    set(_log4cpp_cxx17 TRUE)
    foreach(_loop_var IN LISTS _log4cpp_Priority)
        string(STRIP "${_loop_var}" _file_line)
        if("throw(std::invalid_argument);" STREQUAL "${_file_line}")
            set(_log4cpp_cxx17 FALSE)
        endif()
    endforeach()
    if(${_log4cpp_cxx17})
        set(LOG4CPP_READY_FOR_CXX17 TRUE)
    endif()
endif()

set(LOG4CPP_NAMES log4cpp)
find_library(LOG4CPP_LIBRARY
    NAMES ${LOG4CPP_NAMES}
    HINTS ${PC_LOG4CPP_LIBDIR}
    PATHS ${LOG4CPP_ROOT_USER_PROVIDED}/lib
        ${LOG4CPP_ROOT_USER_PROVIDED}/lib64
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
)

if(LOG4CPP_INCLUDE_DIR AND LOG4CPP_LIBRARY)
    set(LOG4CPP_FOUND TRUE)
    set(LOG4CPP_LIBRARIES ${LOG4CPP_LIBRARY} CACHE INTERNAL "" FORCE)
    set(LOG4CPP_INCLUDE_DIRS ${LOG4CPP_INCLUDE_DIR} CACHE INTERNAL "" FORCE)
else()
    set(LOG4CPP_FOUND FALSE CACHE INTERNAL "" FORCE)
    set(LOG4CPP_LIBRARY "" CACHE INTERNAL "" FORCE)
    set(LOG4CPP_LIBRARIES "" CACHE INTERNAL "" FORCE)
    set(LOG4CPP_INCLUDE_DIR "" CACHE INTERNAL "" FORCE)
    set(LOG4CPP_INCLUDE_DIRS "" CACHE INTERNAL "" FORCE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LOG4CPP DEFAULT_MSG LOG4CPP_INCLUDE_DIRS LOG4CPP_LIBRARIES)

set_package_properties(LOG4CPP PROPERTIES
    URL "http://log4cpp.sourceforge.net/"
)

if(LOG4CPP_FOUND AND PC_LOG4CPP_VERSION)
    set(LOG4CPP_VERSION ${PC_LOG4CPP_VERSION})
endif()

if(LOG4CPP_FOUND AND LOG4CPP_VERSION)
    if(LOG4CPP_READY_FOR_CXX17)
        set_package_properties(LOG4CPP PROPERTIES
            DESCRIPTION "Library of C++ classes for flexible logging (found: v${LOG4CPP_VERSION}, C++17-ready)"
        )
    else()
        set_package_properties(LOG4CPP PROPERTIES
            DESCRIPTION "Library of C++ classes for flexible logging (found: v${LOG4CPP_VERSION})"
        )
    endif()
else()
    set_package_properties(LOG4CPP PROPERTIES
        DESCRIPTION "Library of C++ classes for flexible logging"
    )
endif()

if(LOG4CPP_FOUND AND NOT TARGET Log4cpp::log4cpp)
    add_library(Log4cpp::log4cpp SHARED IMPORTED)
    set_target_properties(Log4cpp::log4cpp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LOG4CPP_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LOG4CPP_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LOG4CPP_LIBRARIES}"
    )
endif()

mark_as_advanced(LOG4CPP_LIBRARIES LOG4CPP_INCLUDE_DIRS)
