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

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_TELEORBIT teleorbit)

FIND_PATH(
    TELEORBIT_INCLUDE_DIRS
    NAMES teleorbit/api.h
    HINTS $ENV{TELEORBIT_DIR}/include
        ${PC_TELEORBIT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    TELEORBIT_LIBRARIES
    NAMES gnuradio-teleorbit
    HINTS $ENV{TELEORBIT_DIR}/lib
        ${PC_TELEORBIT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(TELEORBIT DEFAULT_MSG TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)
MARK_AS_ADVANCED(TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)
