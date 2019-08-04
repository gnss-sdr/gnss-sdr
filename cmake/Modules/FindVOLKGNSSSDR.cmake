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

#
# Provides the following imported target:
# Volkgnsssdr::volkgnsssdr
#


########################################################################
# Find VOLK (Vector-Optimized Library of Kernels) GNSS-SDR library
########################################################################
if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_VOLK_GNSSSDR QUIET volk_gnsssdr)

find_path(VOLK_GNSSSDR_INCLUDE_DIRS
    NAMES volk_gnsssdr/volk_gnsssdr.h
    HINTS $ENV{VOLK_GNSSSDR_DIR}/include
          ${PC_VOLK_GNSSSDR_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
          ${GNURADIO_INSTALL_PREFIX}/include
          ${VOLKGNSSSDR_ROOT}/include
          $ENV{VOLKGNSSSDR_ROOT}/include
)

find_library(VOLK_GNSSSDR_LIBRARIES
    NAMES volk_gnsssdr
    HINTS $ENV{VOLK_GNSSSDR_DIR}/lib
          ${PC_VOLK_GNSSSDR_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          ${GNURADIO_INSTALL_PREFIX}/lib
          ${VOLKGNSSSDR_ROOT}/lib
          $ENV{VOLKGNSSSDR_ROOT}/lib
          ${VOLKGNSSSDR_ROOT}/lib64
          $ENV{VOLKGNSSSDR_ROOT}/lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(VOLKGNSSSDR DEFAULT_MSG VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)
mark_as_advanced(VOLK_GNSSSDR_LIBRARIES VOLK_GNSSSDR_INCLUDE_DIRS)

if(PC_VOLK_GNSSSDR_VERSION)
    set(VOLKGNSSSDR_VERSION ${PC_VOLK_GNSSSDR_VERSION})
endif()

if(VOLKGNSSSDR_FOUND AND VOLKGNSSSDR_VERSION)
    set_package_properties(VOLKGNSSSDR PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels for GNSS-SDR (found: v${VOLKGNSSSDR_VERSION})."
    )
else()
    set_package_properties(VOLKGNSSSDR PROPERTIES
        DESCRIPTION "Vector-Optimized Library of Kernels for GNSS-SDR."
    )
endif()

if(VOLKGNSSSDR_FOUND AND NOT TARGET Volkgnsssdr::volkgnsssdr)
    add_library(Volkgnsssdr::volkgnsssdr SHARED IMPORTED)
    set_target_properties(Volkgnsssdr::volkgnsssdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${VOLK_GNSSSDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${VOLK_GNSSSDR_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${VOLK_GNSSSDR_LIBRARIES}"
    )
endif()
