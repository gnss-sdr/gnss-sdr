# Copyright (C) 2015-2018 (see AUTHORS file for a list of contributors)
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


FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_ORC "orc-0.4 > 0.4.22")

FIND_PROGRAM(ORCC_EXECUTABLE orcc
             HINTS ${PC_ORC_TOOLSDIR}
	     PATHS ${ORC_ROOT}/bin ${CMAKE_INSTALL_PREFIX}/bin)

FIND_PATH(ORC_INCLUDE_DIR NAMES orc/orc.h
          HINTS ${PC_ORC_INCLUDEDIR}
	  PATHS ${ORC_ROOT}/include/orc-0.4 ${CMAKE_INSTALL_PREFIX}/include/orc-0.4)


FIND_PATH(ORC_LIBRARY_DIR NAMES ${CMAKE_SHARED_LIBRARY_PREFIX}orc-0.4${CMAKE_SHARED_LIBRARY_SUFFIX}
          HINTS ${PC_ORC_LIBDIR}
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
                /usr/lib/hppa-linux-gnu
                /usr/lib/s390x-linux-gnu
                /usr/lib64
                /usr/lib
	  PATHS ${ORC_ROOT}/lib${LIB_SUFFIX} ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX})

FIND_LIBRARY(ORC_LIB orc-0.4
             HINTS ${PC_ORC_LIBRARY_DIRS}
	     PATHS ${ORC_ROOT}/lib${LIB_SUFFIX} ${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX})

LIST(APPEND ORC_LIBRARY
     ${ORC_LIB}
)


SET(ORC_INCLUDE_DIRS ${ORC_INCLUDE_DIR})
SET(ORC_LIBRARIES ${ORC_LIBRARY})
SET(ORC_LIBRARY_DIRS ${ORC_LIBRARY_DIR})

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ORC "orc files" ORC_LIBRARY ORC_INCLUDE_DIR ORCC_EXECUTABLE)

mark_as_advanced(ORC_INCLUDE_DIR ORC_LIBRARY ORCC_EXECUTABLE)
