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
# Gnuradio::iio
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_IIO gnuradio-iio)

find_path(IIO_INCLUDE_DIRS
    NAMES gnuradio/iio/api.h
    HINTS $ENV{IIO_DIR}/include
          ${PC_IIO_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          ${GRIIO_ROOT}/include
          $ENV{GRIIO_ROOT}/include
)

if(IIO_INCLUDE_DIRS)
    set(GR_IIO_INCLUDE_HAS_GNURADIO TRUE)
else()
    find_path(IIO_INCLUDE_DIRS
        NAMES iio/api.h
        HINTS $ENV{IIO_DIR}/include
              ${PC_IIO_INCLUDEDIR}
        PATHS ${CMAKE_INSTALL_PREFIX}/include
              /usr/local/include
              /usr/include
              ${GRIIO_ROOT}/include
              $ENV{GRIIO_ROOT}/include
    )
    set(GR_IIO_INCLUDE_HAS_GNURADIO FALSE)
endif()

find_library(IIO_LIBRARIES
    NAMES gnuradio-iio
    HINTS $ENV{IIO_DIR}/lib
          ${PC_IIO_LIBDIR}
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
          ${GRIIO_ROOT}/lib
          $ENV{GRIIO_ROOT}/lib
          ${GRIIO_ROOT}/lib64
          $ENV{GRIIO_ROOT}/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRIIO DEFAULT_MSG IIO_LIBRARIES IIO_INCLUDE_DIRS)

if(PC_IIO_VERSION)
    set(GRIIO_VERSION ${PC_IIO_VERSION})
endif()

set_package_properties(GRIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/gr-iio"
)
if(GRIIO_FOUND AND GRIIO_VERSION)
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio (found: v${GRIIO_VERSION})"
    )
else()
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio"
    )
endif()

if(GRIIO_FOUND AND NOT TARGET Gnuradio::iio)
    add_library(Gnuradio::iio SHARED IMPORTED)
    set_target_properties(Gnuradio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${IIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${IIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${IIO_LIBRARIES}"
    )
endif()

mark_as_advanced(IIO_LIBRARIES IIO_INCLUDE_DIRS GR_IIO_INCLUDE_HAS_GNURADIO)
