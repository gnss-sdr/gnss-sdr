# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Tries to find gr-pocketsdr.
#
# Usage of this module as follows:
#
# find_package(GRPOCKETSDR)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GRPOCKETSDR_ROOT Set this variable to the root installation of
# gr-pocketsdr if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GRPOCKETSDR_FOUND System has gr-pocketsdr libs/headers
# GRPOCKETSDR_LIBRARIES The gr-pocketsdr libraries (gnuradio-pocketsdr)
# GRPOCKETSDR_INCLUDE_DIR The location of gr-pocketsdr headers
#
# Provides the following imported target:
# Gnuradio::pocketsdr
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(GRPOCKETSDR_PKG QUIET gnuradio-pocketsdr)

if(NOT GRPOCKETSDR_ROOT)
    set(GRPOCKETSDR_ROOT_USER_DEFINED /usr)
else()
    set(GRPOCKETSDR_ROOT_USER_DEFINED ${GRPOCKETSDR_ROOT})
endif()
if(DEFINED ENV{GRPOCKETSDR_ROOT})
    set(GRPOCKETSDR_ROOT_USER_DEFINED
        ${GRPOCKETSDR_ROOT_USER_DEFINED}
        $ENV{GRPOCKETSDR_ROOT}
    )
endif()

find_path(GRPOCKETSDR_INCLUDE_DIR
    NAMES
        gnuradio/pocketsdr/pocketsdr_source.h
        gnuradio/pocketsdr/api.h
    HINTS
        ${GRPOCKETSDR_PKG_INCLUDEDIR}
    PATHS
        ${GRPOCKETSDR_ROOT_USER_DEFINED}/include
        ${GNSSSDR_INCLUDE_PATHS}
)

find_library(GRPOCKETSDR_LIBRARIES
    NAMES
        gnuradio-pocketsdr
    HINTS
        ${GRPOCKETSDR_PKG_LIBDIR}
    PATHS
        ${GRPOCKETSDR_ROOT_USER_DEFINED}/lib
        ${GRPOCKETSDR_ROOT_USER_DEFINED}/lib64
        ${GRPOCKETSDR_ROOT_USER_DEFINED}/lib/x86_64-linux-gnu
        ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRPOCKETSDR DEFAULT_MSG GRPOCKETSDR_LIBRARIES GRPOCKETSDR_INCLUDE_DIR)

set_package_properties(GRPOCKETSDR PROPERTIES
    URL "https://github.com/tomojitakasu/PocketSDR"
    DESCRIPTION "Pocket SDR FE GNU Radio blocks"
)

if(GRPOCKETSDR_FOUND AND NOT TARGET Gnuradio::pocketsdr)
    add_library(Gnuradio::pocketsdr SHARED IMPORTED)
    set_target_properties(Gnuradio::pocketsdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GRPOCKETSDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GRPOCKETSDR_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${GRPOCKETSDR_LIBRARIES}"
    )
    message(STATUS "The (optional) gr-pocketsdr module has been found.")
endif()

mark_as_advanced(GRPOCKETSDR_LIBRARIES GRPOCKETSDR_INCLUDE_DIR)
