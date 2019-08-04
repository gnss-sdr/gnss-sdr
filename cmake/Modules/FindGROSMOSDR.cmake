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

# Tries to find gr-osmosdr.
#
# Usage of this module as follows:
#
# find_package(GROSMOSDR)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GrOsmoSDR_ROOT_DIR Set this variable to the root installation of
# gr-osmosdr if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GROSMOSDR_FOUND System has gr-osmosdr libs/headers
# GROSMOSDR_LIBRARIES The gr-osmosdr libraries (gnuradio-osmosdr)
# GROSMOSDR_INCLUDE_DIR The location of gr-osmosdr headers
#
# Provides the following imported target:
# Gnuradio::osmosdr
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(GROSMOSDR_PKG gnuradio-osmosdr)

find_path(GROSMOSDR_INCLUDE_DIR
  NAMES
    osmosdr/source.h
    osmosdr/api.h
  PATHS
    /usr/include
    /usr/local/include
    /opt/local/include
    ${GROSMOSDR_ROOT}/include
    $ENV{GROSMOSDR_ROOT}/include
    ${GROSMOSDR_PKG_INCLUDEDIR}
)

find_library(GROSMOSDR_LIBRARIES
  NAMES gnuradio-osmosdr
  PATHS
    /usr/lib
    /usr/local/lib
    /opt/local/lib
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
    ${GROSMOSDR_ROOT}/lib
    $ENV{GROSMOSDR_ROOT}/lib
    ${GROSMOSDR_ROOT}/lib64
    $ENV{GROSMOSDR_ROOT}/lib64
    ${GROSMOSDR_PKG_LIBDIR}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GROSMOSDR DEFAULT_MSG GROSMOSDR_LIBRARIES GROSMOSDR_INCLUDE_DIR)

if(GROSMOSDR_PKG_VERSION)
    set(GROSMOSDR_VERSION_AUX ${GROSMOSDR_PKG_VERSION})
    string(REGEX REPLACE "^v" "" GROSMOSDR_VERSION ${GROSMOSDR_VERSION_AUX})
endif()

set_package_properties(GROSMOSDR PROPERTIES
    URL "https://osmocom.org/projects/gr-osmosdr/wiki"
)

if(GROSMOSDR_FOUND AND GROSMOSDR_VERSION)
    set_package_properties(GROSMOSDR PROPERTIES
        DESCRIPTION "osmocom GNU Radio blocks (found: v${GROSMOSDR_VERSION})"
    )
else()
    set_package_properties(GROSMOSDR PROPERTIES
        DESCRIPTION "osmocom GNU Radio blocks"
    )
endif()

if(GROSMOSDR_FOUND AND NOT TARGET Gnuradio::osmosdr)
    add_library(Gnuradio::osmosdr SHARED IMPORTED)
    set_target_properties(Gnuradio::osmosdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GROSMOSDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GROSMOSDR_INCLUDE_DIR};${GROSMOSDR_INCLUDE_DIR}/osmosdr"
        INTERFACE_LINK_LIBRARIES "${GROSMOSDR_LIBRARIES}"
    )
endif()

mark_as_advanced(GROSMOSDR_LIBRARIES GROSMOSDR_INCLUDE_DIR)
