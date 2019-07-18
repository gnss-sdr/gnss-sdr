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

########################################################################
# Find  GR-DBFCTTC Module
########################################################################

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_GR_DBFCTTC gr-dbfcttc)

find_path(
    GR_DBFCTTC_INCLUDE_DIRS
    NAMES dbfcttc/api.h
    HINTS $ENV{GR_DBFCTTC_DIR}/include
          ${PC_GR_DBFCTTC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/include
          /usr/local/include
          ${GRDBFCTTC_ROOT}/include
          $ENV{GRDBFCTTC_ROOT}/include
)

find_library(
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
          ${GRDBFCTTC_ROOT}/lib
          $ENV{GRDBFCTTC_ROOT}/lib
          ${GRDBFCTTC_ROOT}/lib64
          $ENV{GRDBFCTTC_ROOT}/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRDBFCTTC DEFAULT_MSG GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)

if(GRDBFCTTC_FOUND AND NOT TARGET Gnuradio::dbfcttc)
    add_library(Gnuradio::dbfcttc SHARED IMPORTED)
    set_target_properties(Gnuradio::dbfcttc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GR_DBFCTTC_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GR_DBFCTTC_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${GR_DBFCTTC_LIBRARIES}"
    )
endif()

mark_as_advanced(GR_DBFCTTC_LIBRARIES GR_DBFCTTC_INCLUDE_DIRS)
