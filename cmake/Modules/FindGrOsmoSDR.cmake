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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.

# Tries to find gr-osmosdr.
#
# Usage of this module as follows:
#
# find_package(GrOsmoSDR)
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

if(NOT GROSMOSDR_FOUND)
  pkg_check_modules (GROSMOSDR_PKG gnuradio-osmosdr)
  find_path(GROSMOSDR_INCLUDE_DIR 
    NAMES osmosdr/source.h
	  osmosdr/api.h
    PATHS
    ${GROSMOSDR_PKG_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
  )

 find_library(GROSMOSDR_LIBRARIES 
    NAMES gnuradio-osmosdr
    PATHS
    ${GROSMOSDR_PKG_LIBRARY_DIRS}
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

  if(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)
    set(GROSMOSDR_FOUND TRUE CACHE INTERNAL "gnuradio-osmosdr found")
    message(STATUS "Found gnuradio-osmosdr: ${GROSMOSDR_INCLUDE_DIR}, ${GROSMOSDR_LIBRARIES}")
  else(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)
    set(GROSMOSDR_FOUND FALSE CACHE INTERNAL "gnuradio-osmosdr found")
    message(STATUS "gnuradio-osmosdr not found.")
  endif(GROSMOSDR_INCLUDE_DIR AND GROSMOSDR_LIBRARIES)

mark_as_advanced(GROSMOSDR_INCLUDE_DIR GROSMOSDR_LIBRARIES)

endif(NOT GROSMOSDR_FOUND)
