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

# - Find gpstk library
# Find the native gpstk includes and library
# This module defines
#  GPSTK_INCLUDE_DIR, where to find Rinex3ObsBase.hpp, etc.
#  GPSTK_FOUND, If false, do not try to use GPSTK.
#  GPSTK_LIBRARY, where to find the GPSTK library.

find_path(GPSTK_INCLUDE_DIR gpstk/Rinex3ObsBase.hpp
    HINTS /usr/include
          /usr/local/include
          /opt/local/include
          ${GPSTK_ROOT}/include
          $ENV{GPSTK_ROOT}/include
)

set(GPSTK_NAMES ${GPSTK_NAMES} gpstk libgpstk)

include(GNUInstallDirs)

find_library(GPSTK_LIBRARY NAMES ${GPSTK_NAMES}
    HINTS /usr/lib
          /usr/local/lib
          /usr/${CMAKE_INSTALL_LIBDIR}
          /usr/local/${CMAKE_INSTALL_LIBDIR}
          /opt/local/lib
          ${GPSTK_ROOT}/${CMAKE_INSTALL_LIBDIR}
          $ENV{GPSTK_ROOT}/${CMAKE_INSTALL_LIBDIR}
)

# handle the QUIETLY and REQUIRED arguments and set GPSTK_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GPSTK DEFAULT_MSG GPSTK_LIBRARY GPSTK_INCLUDE_DIR)
mark_as_advanced(GPSTK_LIBRARY GPSTK_INCLUDE_DIR)

if(GPSTK_FOUND AND NOT TARGET Gpstk::gpstk)
    add_library(Gpstk::gpstk SHARED IMPORTED)
    set_target_properties(Gpstk::gpstk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPSTK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPSTK_INCLUDE_DIR};${GPSTK_INCLUDE_DIR}/gpstk"
        INTERFACE_LINK_LIBRARIES "${GPSTK_LIBRARY}"
    )
endif()
