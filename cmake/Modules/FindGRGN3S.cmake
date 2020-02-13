# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

########################################################################
# Find  GR-GN3S Module
########################################################################

#
# Provides the following imported target:
# Gnuradio::gn3s
#

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_GR_GN3S gr-gn3s)

find_path(
    GR_GN3S_INCLUDE_DIRS
    NAMES gn3s/gn3s_api.h
    HINTS ${PC_GR_GN3S_INCLUDEDIR}
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
          ${CMAKE_INSTALL_PREFIX}/include
          ${GRGN3S_ROOT}/include
          $ENV{GRGN3S_ROOT}/include
          $ENV{GR_GN3S_DIR}/include
)

find_library(
    GR_GN3S_LIBRARIES
    NAMES gr-gn3s
    HINTS ${PC_GR_GN3S_LIBDIR}
    PATHS /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          ${GRGN3S_ROOT}/lib
          $ENV{GRGN3S_ROOT}/lib
          ${GRGN3S_ROOT}/lib64
          $ENV{GRGN3S_ROOT}/lib64
          $ENV{GR_GN3S_DIR}/lib
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
