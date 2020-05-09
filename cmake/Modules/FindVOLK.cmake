# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

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

if(VOLK_FOUND AND NOT TARGET Volk::volk)
    add_library(Volk::volk SHARED IMPORTED)
    set_target_properties(Volk::volk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${VOLK_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${VOLK_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${VOLK_LIBRARIES}"
    )
endif()
