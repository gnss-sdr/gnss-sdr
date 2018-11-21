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
# Find VOLK (Vector-Optimized Library of Kernels) GNSS-SDR library
########################################################################

include(FindPkgConfig)
pkg_check_modules(PC_VOLK_GNSSSDR volk_gnsssdr)

find_path(
    VOLK_GNSSSDR_INCLUDE_DIRS
    NAMES volk_gnsssdr/volk_gnsssdr.h
    HINTS $ENV{VOLK_GNSSSDR_DIR}/include
        ${PC_VOLK_GNSSSDR_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
          ${GNURADIO_INSTALL_PREFIX}/include
)

find_library(
    VOLK_GNSSSDR_LIBRARIES
    NAMES volk_gnsssdr
    HINTS $ENV{VOLK_GNSSSDR_DIR}/lib
        ${PC_VOLK_GNSSSDR_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          ${GNURADIO_INSTALL_PREFIX}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VOLKGNSSSDR DEFAULT_MSG VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
mark_as_advanced(VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
