# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
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

# Tries to find libosmosdr.
#
# Usage of this module as follows:
#
# find_package(LIBOSMOSDR)
#
#
# Variables defined by this module:
#
# LIBOSMOSDR_FOUND System has libosmosdr libs/headers
# LIBOSMOSDR_LIBRARIES The libosmosdr libraries
# LIBOSMOSDR_INCLUDE_DIR The location of libosmosdr headers

pkg_check_modules(LIBOSMOSDR_PKG libosmosdr)
find_path(LIBOSMOSDR_INCLUDE_DIR NAMES osmosdr.h
  PATHS
  ${LIBOSMOSDR_PKG_INCLUDE_DIRS}
  /usr/include
  /usr/local/include
)

find_library(LIBOSMOSDR_LIBRARIES NAMES osmosdr
  PATHS
    ${LIBOSMOSDR_PKG_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
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
  )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBOSMOSDR DEFAULT_MSG LIBOSMOSDR_INCLUDE_DIR LIBOSMOSDR_LIBRARIES)
mark_as_advanced(LIBOSMOSDR_INCLUDE_DIR LIBOSMOSDR_LIBRARIES)
