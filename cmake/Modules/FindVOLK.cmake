# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Volk::volk
#

########################################################################
# Find VOLK (Vector-Optimized Library of Kernels)
########################################################################
if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(PC_VOLK QUIET volk)


########################################################################
# Build candidate root list
########################################################################
set(_VOLK_ROOT_HINTS)

if(VOLK_ROOT)
    list(APPEND _VOLK_ROOT_HINTS "${VOLK_ROOT}")
endif()

if(DEFINED ENV{VOLK_ROOT} AND NOT "$ENV{VOLK_ROOT}" STREQUAL "")
    list(APPEND _VOLK_ROOT_HINTS "$ENV{VOLK_ROOT}")
endif()

if(DEFINED ENV{VOLK_DIR} AND NOT "$ENV{VOLK_DIR}" STREQUAL "")
    list(APPEND _VOLK_ROOT_HINTS "$ENV{VOLK_DIR}")
endif()

if(CMAKE_INSTALL_PREFIX)
    list(APPEND _VOLK_ROOT_HINTS "${CMAKE_INSTALL_PREFIX}")
endif()

if(DEFINED _VOLK_ROOT_HINTS)
    list(REMOVE_DUPLICATES _VOLK_ROOT_HINTS)
endif()


########################################################################
# Search paths for headers and libraries
########################################################################
set(_VOLK_INCLUDE_HINTS)
set(_VOLK_LIBRARY_HINTS)

if(PC_VOLK_INCLUDEDIR)
    list(APPEND _VOLK_INCLUDE_HINTS "${PC_VOLK_INCLUDEDIR}")
endif()

if(PC_VOLK_LIBDIR)
    list(APPEND _VOLK_LIBRARY_HINTS "${PC_VOLK_LIBDIR}")
endif()

foreach(_volk_prefix ${_VOLK_ROOT_HINTS})
    list(APPEND _VOLK_INCLUDE_HINTS "${_volk_prefix}/include")
    list(APPEND _VOLK_LIBRARY_HINTS
        "${_volk_prefix}/lib"
        "${_volk_prefix}/lib64"
    )
endforeach()

list(APPEND _VOLK_INCLUDE_HINTS ${GNSSSDR_INCLUDE_PATHS})
list(APPEND _VOLK_LIBRARY_HINTS ${GNSSSDR_LIB_PATHS})

list(REMOVE_DUPLICATES _VOLK_INCLUDE_HINTS)
list(REMOVE_DUPLICATES _VOLK_LIBRARY_HINTS)


########################################################################
# Find headers and library
########################################################################
find_path(VOLK_INCLUDE_DIR
    NAMES volk/volk.h
    HINTS ${PC_VOLK_INCLUDEDIR}
    PATHS ${_VOLK_INCLUDE_HINTS}
)

find_library(VOLK_LIBRARY
    NAMES volk
    HINTS ${PC_VOLK_LIBDIR}
    PATHS ${_VOLK_LIBRARY_HINTS}
)


########################################################################
# Standard result handling
########################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VOLK DEFAULT_MSG
    VOLK_LIBRARY
    VOLK_INCLUDE_DIR
)


########################################################################
# Determine version
########################################################################
if(PC_VOLK_VERSION)
    set(VOLK_VERSION "${PC_VOLK_VERSION}")
endif()

if(VOLK_FOUND AND NOT VOLK_VERSION)
    get_filename_component(_VOLK_LIB_DIR "${VOLK_LIBRARY}" DIRECTORY)
    if(EXISTS "${_VOLK_LIB_DIR}/cmake/volk/VolkConfigVersion.cmake")
        if(DEFINED PACKAGE_VERSION)
            set(_VOLK_PACKAGE_VERSION_WAS_DEFINED TRUE)
            set(_VOLK_OLD_PACKAGE_VERSION "${PACKAGE_VERSION}")
        else()
            set(_VOLK_PACKAGE_VERSION_WAS_DEFINED FALSE)
        endif()

        unset(PACKAGE_VERSION)
        set(PACKAGE_FIND_VERSION_MAJOR 0)
        include("${_VOLK_LIB_DIR}/cmake/volk/VolkConfigVersion.cmake")

        if(PACKAGE_VERSION)
            set(VOLK_VERSION "${PACKAGE_VERSION}")
        endif()

        unset(PACKAGE_VERSION)
        unset(PACKAGE_FIND_VERSION_MAJOR)

        if(_VOLK_PACKAGE_VERSION_WAS_DEFINED)
            set(PACKAGE_VERSION "${_VOLK_OLD_PACKAGE_VERSION}")
        else()
            unset(PACKAGE_VERSION)
        endif()

        unset(_VOLK_PACKAGE_VERSION_WAS_DEFINED)
        unset(_VOLK_OLD_PACKAGE_VERSION)
    endif()
endif()


########################################################################
# Package metadata
########################################################################
set_package_properties(VOLK PROPERTIES
    URL "https://www.libvolk.org"
)

if(VOLK_FOUND AND VOLK_VERSION)
    set_package_properties(VOLK PROPERTIES
        DESCRIPTION
        "Vector-Optimized Library of Kernels (found: v${VOLK_VERSION})"
    )
else()
    set_package_properties(VOLK PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels"
    )
endif()


########################################################################
# ORC support
########################################################################
if(NOT ORC_FOUND)
    find_package(ORC QUIET)
endif()

set(VOLK_LINK_LIBRARIES)
set(VOLK_INTERFACE_INCLUDE_DIRS "${VOLK_INCLUDE_DIR}")

if(ORC_FOUND AND ORC_LIBRARIES_STATIC)
    list(APPEND VOLK_LINK_LIBRARIES ${ORC_LIBRARIES_STATIC})
    if(VOLK_VERSION AND VOLK_VERSION VERSION_LESS "2.3.0")
        if(ORC_INCLUDE_DIRS)
            list(APPEND VOLK_INTERFACE_INCLUDE_DIRS ${ORC_INCLUDE_DIRS})
        endif()
    endif()
endif()

list(REMOVE_DUPLICATES VOLK_INTERFACE_INCLUDE_DIRS)
if(DEFINED VOLK_LINK_LIBRARIES)
    list(REMOVE_DUPLICATES VOLK_LINK_LIBRARIES)
endif()


########################################################################
# Compatibility variables
########################################################################
set(VOLK_LIBRARIES "${VOLK_LIBRARY}")
set(VOLK_INCLUDE_DIRS "${VOLK_INTERFACE_INCLUDE_DIRS}")

mark_as_advanced(
    VOLK_LIBRARY
    VOLK_INCLUDE_DIR
    VOLK_LIBRARIES
    VOLK_INCLUDE_DIRS
    VOLK_VERSION
)


########################################################################
# Define imported target
########################################################################
if(VOLK_FOUND AND NOT TARGET Volk::volk)
    add_library(Volk::volk UNKNOWN IMPORTED)
    set_target_properties(Volk::volk PROPERTIES
        IMPORTED_LOCATION "${VOLK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${VOLK_INTERFACE_INCLUDE_DIRS}"
    )
    if(VOLK_LINK_LIBRARIES)
        set_target_properties(Volk::volk PROPERTIES
            INTERFACE_LINK_LIBRARIES "${VOLK_LINK_LIBRARIES}"
        )
    endif()
endif()


########################################################################
# Cleanup internal variables
########################################################################
unset(_VOLK_ROOT_HINTS)
unset(_VOLK_INCLUDE_HINTS)
unset(_VOLK_LIBRARY_HINTS)
unset(_VOLK_LIB_DIR)
unset(_volk_prefix)
unset(VOLK_INTERFACE_INCLUDE_DIRS)
unset(VOLK_LINK_LIBRARIES)
