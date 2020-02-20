# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

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

pkg_check_modules(PC_VOLK_GNSSSDR QUIET volk_gnsssdr)

find_path(VOLK_GNSSSDR_INCLUDE_DIRS
    NAMES volk_gnsssdr/volk_gnsssdr.h
    HINTS ${PC_VOLK_GNSSSDR_INCLUDEDIR}
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
          ${GNURADIO_INSTALL_PREFIX}/include
          ${VOLKGNSSSDR_ROOT}/include
          $ENV{VOLKGNSSSDR_ROOT}/include
          $ENV{VOLK_GNSSSDR_DIR}/include
)

find_library(VOLK_GNSSSDR_LIBRARIES
    NAMES volk_gnsssdr
    HINTS ${PC_VOLK_GNSSSDR_LIBDIR}
    PATHS /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
          ${GNURADIO_INSTALL_PREFIX}/lib
          ${VOLKGNSSSDR_ROOT}/lib
          $ENV{VOLKGNSSSDR_ROOT}/lib
          ${VOLKGNSSSDR_ROOT}/lib64
          $ENV{VOLKGNSSSDR_ROOT}/lib64
          $ENV{VOLK_GNSSSDR_DIR}/lib
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
