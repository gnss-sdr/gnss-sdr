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

########################################################################
# Find  GR-DBFCTTC Module
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GR_DBFCTTC gr-dbfcttc)

FIND_PATH(
    GR_DBFCTTC_INCLUDE_DIRS
    NAMES dbfcttc/api.h
    HINTS $ENV{GR_DBFCTTC_DIR}/include
          ${PC_GR_DBFCTTC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/include 
          /usr/local/include
)

FIND_LIBRARY(
    GR_DBFCTTC_LIBRARIES
    NAMES gnuradio-dbfcttc
    HINTS $ENV{GR_DBFCTTC_DIR}/lib
          ${PC_GR_DBFCTTC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_DBFCTTC DEFAULT_MSG GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)
