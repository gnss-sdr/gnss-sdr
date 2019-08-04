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
# Find  GR-GN3S Module
########################################################################

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_GR_GN3S gr-gn3s)

find_path(
    GR_GN3S_INCLUDE_DIRS
    NAMES gn3s/gn3s_api.h
    HINTS $ENV{GR_GN3S_DIR}/include
          ${PC_GR_GN3S_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          ${GRGN3S_ROOT}/include
          $ENV{GRGN3S_ROOT}/include
)

find_library(
    GR_GN3S_LIBRARIES
    NAMES gr-gn3s
    HINTS $ENV{GR_GN3S_DIR}/lib
          ${PC_GR_GN3S_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          ${GRGN3S_ROOT}/lib
          $ENV{GRGN3S_ROOT}/lib
          ${GRGN3S_ROOT}/lib64
          $ENV{GRGN3S_ROOT}/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRGN3S DEFAULT_MSG GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)

if(GRGN3S_FOUND AND NOT TARGET Gnuradio::gn3s)
    add_library(Gnuradio::gn3s SHARED IMPORTED)
    set_target_properties(Gnuradio::gn3s PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GR_GN3S_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GR_GN3S_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${GR_GN3S_LIBRARIES}"
    )
endif()

mark_as_advanced(GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)
