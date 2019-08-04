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
pkg_check_modules(PC_LIBIIO libiio)

find_path(
    LIBIIO_INCLUDE_DIRS
    NAMES iio.h
    HINTS $ENV{LIBIIO_DIR}/include
          ${PC_LIBIIO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          /opt/local/include
          ${LIBIIO_ROOT}/include
          $ENV{LIBIIO_ROOT}/include
)

find_library(
    LIBIIO_LIBRARIES
    NAMES iio libiio.so.0
    HINTS $ENV{LIBIIO_DIR}/lib
          ${PC_LIBIIO_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
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
          /Library/Frameworks/iio.framework/
          ${LIBIIO_ROOT}/lib
          $ENV{LIBIIO_ROOT}/lib
          ${LIBIIO_ROOT}/lib64
          $ENV{LIBIIO_ROOT}/lib64
)

if(LIBIIO_LIBRARIES AND APPLE)
    if(LIBIIO_LIBRARIES MATCHES "framework")
        set(LIBIIO_LIBRARIES ${LIBIIO_LIBRARIES}/iio)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBIIO DEFAULT_MSG LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)

if(PC_LIBIIO_VERSION)
    set(LIBIIO_VERSION ${PC_LIBIIO_VERSION})
endif()

if(LIBIIO_FOUND AND LIBIIO_VERSION)
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices (found: v${LIBIIO_VERSION})"
    )
else()
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices"
    )
endif()

set_package_properties(LIBIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/libiio"
)

if(LIBIIO_FOUND AND NOT TARGET Iio::iio)
    add_library(Iio::iio SHARED IMPORTED)
    set_target_properties(Iio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBIIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBIIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBIIO_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
