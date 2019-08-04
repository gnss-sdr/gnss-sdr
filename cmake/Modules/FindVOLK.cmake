# Copyright (C) 2011-2019 (see AUTHORS file for a list of contributors)
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

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_VOLK volk QUIET)

find_path(VOLK_INCLUDE_DIRS
    NAMES volk/volk.h
    HINTS $ENV{VOLK_DIR}/include
          ${PC_VOLK_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
          ${CMAKE_INSTALL_PREFIX}/include
          ${VOLK_ROOT}/include
          $ENV{VOLK_ROOT}/include
)

find_library(VOLK_LIBRARIES
    NAMES volk
    HINTS $ENV{VOLK_DIR}/lib
          ${PC_VOLK_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
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
          ${CMAKE_INSTALL_PREFIX}/lib
          ${VOLK_ROOT}/lib
          $ENV{VOLK_ROOT}/lib
          ${VOLK_ROOT}/lib64
          $ENV{VOLK_ROOT}/lib64
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
        include(${VOLK_LIB_DIR}/cmake/volk/VolkConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(VOLK_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

set_package_properties(VOLK PROPERTIES
    URL "http://libvolk.org"
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
