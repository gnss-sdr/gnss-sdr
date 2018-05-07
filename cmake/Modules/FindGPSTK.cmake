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

# - Find gpstk library
# Find the native gpstk includes and library
# This module defines
#  GPSTK_INCLUDE_DIR, where to find Rinex3ObsBase.hpp, etc.
#  GPSTK_LIBRARIES, libraries to link against to use GPSTK.
#  GPSTK_FOUND, If false, do not try to use GPSTK.
# also defined, but not for general use are
#  GPSTK_LIBRARY, where to find the GPSTK library.

FIND_PATH(GPSTK_INCLUDE_DIR Rinex3ObsBase.hpp
          HINTS /usr/include/gpstk
                /usr/local/include/gpstk
                /opt/local/include/gpstk )

SET(GPSTK_NAMES ${GPSTK_NAMES} gpstk libgpstk)
FIND_LIBRARY(GPSTK_LIBRARY NAMES ${GPSTK_NAMES}
             HINTS /usr/lib
                   /usr/local/lib
                   /opt/local/lib )

# handle the QUIETLY and REQUIRED arguments and set GPSTK_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GPSTK  DEFAULT_MSG  GPSTK_LIBRARY  GPSTK_INCLUDE_DIR)

IF(GPSTK_FOUND)
  SET( GPSTK_LIBRARIES ${GPSTK_LIBRARY} )
ENDIF(GPSTK_FOUND)

MARK_AS_ADVANCED(GPSTK_INCLUDE_DIR GPSTK_LIBRARY)
