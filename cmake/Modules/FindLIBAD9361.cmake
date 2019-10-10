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
# Iio::iio
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_LIBAD9361 libad9361)

find_path(
    LIBAD9361_INCLUDE_DIRS
    NAMES ad9361.h
    HINTS ${PC_LIBAD9361_INCLUDEDIR}
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
          ${CMAKE_INSTALL_PREFIX}/include
          ${LIBAD9361_ROOT}/include
          $ENV{LIBAD9361_ROOT}/include
          $ENV{LIBAD9361_DIR}/include
)

find_library(
    LIBAD9361_LIBRARIES
    NAMES ad9361
    HINTS ${PC_LIBAD9361_LIBDIR}
    PATHS /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
          /usr/lib/i386-linux-gnu
          /usr/lib/alpha-linux-gnu
          /usr/lib/aarch64-linux-gnu
          /usr/lib/arm-linux-gnueabi
          /usr/lib/arm-linux-gnueabihf
          /usr/lib/hppa-linux-gnu
          /usr/lib/i686-gnu
          /usr/lib/i686-linux-gnu
          /usr/lib/x86_64-kfreebsd-gnu
          /usr/lib/i686-kfreebsd-gnu
          /usr/lib/m68k-linux-gnu
          /usr/lib/mips-linux-gnu
          /usr/lib/mips64el-linux-gnuabi64
          /usr/lib/mipsel-linux-gnu
          /usr/lib/powerpc-linux-gnu
          /usr/lib/powerpc-linux-gnuspe
          /usr/lib/powerpc64-linux-gnu
          /usr/lib/powerpc64le-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/lib/sparc64-linux-gnu
          /usr/lib/x86_64-linux-gnux32
          /usr/lib/sh4-linux-gnu
          /usr/lib/riscv64-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
          /Library/Frameworks/ad9361.framework
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          ${LIBAD9361_ROOT}/lib
          $ENV{LIBAD9361_ROOT}/lib
          ${LIBAD9361_ROOT}/lib64
          $ENV{LIBAD9361_ROOT}/lib64
          $ENV{LIBAD9361_DIR}/lib
)

if(LIBAD9361_LIBRARIES AND APPLE)
    if(LIBAD9361_LIBRARIES MATCHES "framework")
        set(LIBAD9361_LIBRARIES ${LIBAD9361_LIBRARIES}/ad9361)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBAD9361 DEFAULT_MSG LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)

if(PC_LIBAD9361_VERSION)
    set(LIBAD9361_VERSION ${PC_LIBAD9361_VERSION})
endif()

if(LIBAD9361_FOUND AND LIBAD9361_VERSION)
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers (found: v${LIBAD9361_VERSION})"
    )
else()
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers"
    )
endif()

set_package_properties(LIBAD9361 PROPERTIES
    URL "https://github.com/analogdevicesinc/libad9361-iio"
)

if(LIBAD9361_FOUND AND NOT TARGET Iio::ad9361)
    add_library(Iio::ad9361 SHARED IMPORTED)
    set_target_properties(Iio::ad9361 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBAD9361_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBAD9361_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBAD9361_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)
