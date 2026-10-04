# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2026 (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Find ORC (Optimized Inner Loops Runtime Compiler)
########################################################################

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
set(FPHSA_NAME_MISMATCHED ON)

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED VOLK_GNSSSDR_LIB_PATHS)
    include(VolkGnsssdrFindPaths)
endif()

pkg_check_modules(PC_ORC QUIET "orc-0.4 > 0.4.22")


########################################################################
# Build candidate root list
########################################################################
set(_ORC_ROOT_HINTS)

if(ORC_ROOT)
    list(APPEND _ORC_ROOT_HINTS "${ORC_ROOT}")
endif()

if(DEFINED ENV{ORC_ROOT} AND NOT "$ENV{ORC_ROOT}" STREQUAL "")
    list(APPEND _ORC_ROOT_HINTS "$ENV{ORC_ROOT}")
endif()

if(CMAKE_INSTALL_PREFIX)
    list(APPEND _ORC_ROOT_HINTS "${CMAKE_INSTALL_PREFIX}")
endif()

if(DEFINED _ORC_ROOT_HINTS)
    list(REMOVE_DUPLICATES _ORC_ROOT_HINTS)
endif()


########################################################################
# Search paths for binaries, headers, and libraries
########################################################################
set(_ORC_BIN_HINTS)
set(_ORC_INCLUDE_HINTS)
set(_ORC_LIBRARY_HINTS)

if(PC_ORC_PREFIX)
    list(APPEND _ORC_BIN_HINTS "${PC_ORC_PREFIX}/bin")
endif()

if(PC_ORC_TOOLSDIR)
    list(APPEND _ORC_BIN_HINTS "${PC_ORC_TOOLSDIR}")
endif()

if(PC_ORC_INCLUDEDIR)
    list(APPEND _ORC_INCLUDE_HINTS "${PC_ORC_INCLUDEDIR}")
endif()

if(PC_ORC_LIBDIR)
    list(APPEND _ORC_LIBRARY_HINTS "${PC_ORC_LIBDIR}")
endif()

if(PC_ORC_LIBRARY_DIRS)
    list(APPEND _ORC_LIBRARY_HINTS ${PC_ORC_LIBRARY_DIRS})
endif()

foreach(_orc_prefix ${_ORC_ROOT_HINTS})
    list(APPEND _ORC_BIN_HINTS "${_orc_prefix}/bin")
    list(APPEND _ORC_INCLUDE_HINTS "${_orc_prefix}/include")
    list(APPEND _ORC_LIBRARY_HINTS
        "${_orc_prefix}/lib"
        "${_orc_prefix}/lib64"
    )
endforeach()

list(APPEND _ORC_BIN_HINTS
    /usr/bin
    /usr/local/bin
)
foreach(_orc_system_prefix ${CMAKE_SYSTEM_PREFIX_PATH})
    list(APPEND _ORC_BIN_HINTS "${_orc_system_prefix}/bin")
endforeach()
list(APPEND _ORC_INCLUDE_HINTS ${VOLK_GNSSSDR_INCLUDE_PATHS})
list(APPEND _ORC_LIBRARY_HINTS ${VOLK_GNSSSDR_LIB_PATHS})

list(REMOVE_DUPLICATES _ORC_BIN_HINTS)
list(REMOVE_DUPLICATES _ORC_INCLUDE_HINTS)
list(REMOVE_DUPLICATES _ORC_LIBRARY_HINTS)


########################################################################
# Find executable, headers, and libraries
########################################################################
find_program(ORCC_EXECUTABLE
    NAMES orcc
    PATHS ${_ORC_BIN_HINTS}
)

find_path(ORC_INCLUDE_DIR
    NAMES orc/orc.h
    HINTS ${PC_ORC_INCLUDEDIR}
    PATHS ${_ORC_INCLUDE_HINTS}
    PATH_SUFFIXES orc-0.4
)

find_library(ORC_LIBRARY
    NAMES orc-0.4
    HINTS ${PC_ORC_LIBDIR} ${PC_ORC_LIBRARY_DIRS}
    PATHS ${_ORC_LIBRARY_HINTS}
)

find_library(ORC_LIBRARY_STATIC
    NAMES
        ${CMAKE_STATIC_LIBRARY_PREFIX}orc-0.4${CMAKE_STATIC_LIBRARY_SUFFIX}
    HINTS ${PC_ORC_LIBDIR} ${PC_ORC_LIBRARY_DIRS}
    PATHS ${_ORC_LIBRARY_HINTS}
)


########################################################################
# Derive library directory and version
########################################################################
if(ORC_LIBRARY)
    get_filename_component(ORC_LIBRARY_DIR "${ORC_LIBRARY}" DIRECTORY)
endif()

if(PC_ORC_VERSION)
    set(ORC_VERSION "${PC_ORC_VERSION}")
endif()


########################################################################
# Compatibility variables
########################################################################
set(ORC_INCLUDE_DIRS "${ORC_INCLUDE_DIR}")
set(ORC_LIBRARIES "${ORC_LIBRARY}")
set(ORC_LIBRARY_DIRS "${ORC_LIBRARY_DIR}")
set(ORC_LIBRARIES_STATIC "${ORC_LIBRARY_STATIC}")


########################################################################
# Standard result handling
########################################################################
include(FindPackageHandleStandardArgs)
if(ENABLE_STATIC_LIBS)
    find_package_handle_standard_args(ORC
        "orc files"
        ORC_LIBRARY
        ORC_INCLUDE_DIR
        ORCC_EXECUTABLE
        ORC_LIBRARY_STATIC
    )
else()
    find_package_handle_standard_args(ORC
        "orc files"
        ORC_LIBRARY
        ORC_INCLUDE_DIR
        ORCC_EXECUTABLE
    )
endif()

if(ORC_FOUND AND NOT TARGET ORC::orc)
    add_library(ORC::orc UNKNOWN IMPORTED)
    set_target_properties(ORC::orc PROPERTIES
        IMPORTED_LOCATION "${ORC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${ORC_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(
    ORCC_EXECUTABLE
    ORC_INCLUDE_DIR
    ORC_INCLUDE_DIRS
    ORC_LIBRARY
    ORC_LIBRARIES
    ORC_LIBRARY_DIR
    ORC_LIBRARY_DIRS
    ORC_LIBRARY_STATIC
    ORC_LIBRARIES_STATIC
    ORC_VERSION
)


########################################################################
# Cleanup internal variables
########################################################################
unset(_ORC_ROOT_HINTS)
unset(_ORC_BIN_HINTS)
unset(_ORC_INCLUDE_HINTS)
unset(_ORC_LIBRARY_HINTS)
unset(_orc_prefix)
unset(_orc_system_prefix)
