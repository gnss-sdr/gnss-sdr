# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Find  GR-DBFCTTC Module
########################################################################

#
# Provides the following imported target:
# Gnuradio::dbfcttc
#

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_GR_DBFCTTC gr-dbfcttc)

if(NOT GRDBFCTTC_ROOT)
    set(GRDBFCTTC_ROOT_USER_DEFINED /usr/local)
else()
    set(GRDBFCTTC_ROOT_USER_DEFINED ${GRDBFCTTC_ROOT})
endif()
if(DEFINED ENV{GRDBFCTTC_ROOT})
    set(GRDBFCTTC_ROOT_USER_DEFINED
        ${GRDBFCTTC_ROOT_USER_DEFINED}
        $ENV{GRDBFCTTC_ROOT}
    )
endif()
if(DEFINED ENV{GR_DBFCTTC_DIR})
    set(GRDBFCTTC_ROOT_USER_DEFINED
        ${GRDBFCTTC_ROOT_USER_DEFINED}
        $ENV{GR_DBFCTTC_DIR}
    )
endif()
set(GRDBFCTTC_ROOT_USER_DEFINED
    ${GRDBFCTTC_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(
    GR_DBFCTTC_INCLUDE_DIRS
    NAMES dbfcttc/api.h
    HINTS ${PC_GR_DBFCTTC_INCLUDEDIR}
    PATHS ${GRDBFCTTC_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(
    GR_DBFCTTC_LIBRARIES
    NAMES gnuradio-dbfcttc
    HINTS ${PC_GR_DBFCTTC_LIBDIR}
    PATHS ${GRDBFCTTC_ROOT_USER_DEFINED}/lib
          ${GRDBFCTTC_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
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
