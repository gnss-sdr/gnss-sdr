# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
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

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_VOLK volk QUIET)

if(NOT VOLK_ROOT)
    set(VOLK_ROOT_USER_PROVIDED /usr)
else()
    set(VOLK_ROOT_USER_PROVIDED ${VOLK_ROOT})
endif()
if(DEFINED ENV{VOLK_ROOT})
    set(VOLK_ROOT_USER_PROVIDED
        ${VOLK_ROOT_USER_PROVIDED}
        $ENV{VOLK_ROOT}
    )
endif()
if(DEFINED ENV{VOLK_DIR})
    set(VOLK_ROOT_USER_PROVIDED
        ${VOLK_ROOT_USER_PROVIDED}
        $ENV{VOLK_DIR}
    )
endif()
set(VOLK_ROOT_USER_PROVIDED
    ${VOLK_ROOT_USER_PROVIDED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(VOLK_INCLUDE_DIRS
    NAMES volk/volk.h
    HINTS ${PC_VOLK_INCLUDEDIR}
    PATHS ${VOLK_ROOT_USER_PROVIDED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(VOLK_LIBRARIES
    NAMES volk
    HINTS ${PC_VOLK_LIBDIR}
    PATHS ${VOLK_ROOT_USER_PROVIDED}/lib
          ${VOLK_ROOT_USER_PROVIDED}/lib64
          ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VOLK DEFAULT_MSG VOLK_LIBRARIES VOLK_INCLUDE_DIRS)

if(PC_VOLK_VERSION)
    set(VOLK_VERSION ${PC_VOLK_VERSION})
endif()

if(NOT VOLK_VERSION)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET VOLK_LIBRARIES 0 FIRST_DIR)
    get_filename_component(VOLK_LIB_DIR ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${VOLK_LIB_DIR}/cmake/volk/VolkConfigVersion.cmake)
        set(PACKAGE_FIND_VERSION_MAJOR 0)
        include(${VOLK_LIB_DIR}/cmake/volk/VolkConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(VOLK_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

set_package_properties(VOLK PROPERTIES
    URL "https://www.libvolk.org"
)

if(VOLK_FOUND AND VOLK_VERSION)
    set_package_properties(VOLK PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels (found: v${VOLK_VERSION})"
    )
else()
    set_package_properties(VOLK PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels"
    )
endif()

mark_as_advanced(VOLK_LIBRARIES VOLK_INCLUDE_DIRS VOLK_VERSION)

if(NOT ORC_FOUND)
    find_package(ORC QUIET)
endif()
if(ORC_LIBRARIES_STATIC)
    set(VOLK_LINK_LIBRARIES ${VOLK_LIBRARIES} ${ORC_LIBRARIES_STATIC})
    set(VOLK_INCLUDE_DIRS ${VOLK_INCLUDE_DIRS} ${ORC_INCLUDE_DIRS})
else()
    set(VOLK_LINK_LIBRARIES ${VOLK_LIBRARIES})
endif()

if(VOLK_FOUND AND NOT TARGET Volk::volk)
    add_library(Volk::volk SHARED IMPORTED)
    set_target_properties(Volk::volk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${VOLK_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${VOLK_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${VOLK_LINK_LIBRARIES}"
    )
endif()
