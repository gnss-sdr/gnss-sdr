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
# Find  GR-GN3S Module
########################################################################

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
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRGN3S DEFAULT_MSG GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)
mark_as_advanced(GR_GN3S_LIBRARIES GR_GN3S_INCLUDE_DIRS)
