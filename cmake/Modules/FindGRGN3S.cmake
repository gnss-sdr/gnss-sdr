# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

########################################################################
# Find  GR-GN3S Module
########################################################################

#
# Provides the following imported target:
# Gnuradio::gn3s
#

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_GR_GN3S gr-gn3s)

if(NOT GRGN3S_ROOT)
    set(GRGN3S_ROOT_USER_DEFINED /usr/local)
else()
    set(GRGN3S_ROOT_USER_DEFINED ${GRGN3S_ROOT})
endif()
if(DEFINED ENV{GRGN3S_ROOT})
    set(GRGN3S_ROOT_USER_DEFINED
        ${GRGN3S_ROOT_USER_DEFINED}
        $ENV{GRGN3S_ROOT}
    )
endif()
if(DEFINED ENV{GR_GN3S_DIR})
    set(GRGN3S_ROOT_USER_DEFINED
        ${GRGN3S_ROOT_USER_DEFINED}
        $ENV{GR_GN3S_DIR}
    )
endif()
set(GRGN3S_ROOT_USER_DEFINED
    ${GRGN3S_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(
    GR_GN3S_INCLUDE_DIRS
    NAMES gn3s/gn3s_api.h
    HINTS ${PC_GR_GN3S_INCLUDEDIR}
    PATHS ${GRGN3S_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(
    GR_GN3S_LIBRARIES
    NAMES gr-gn3s
    HINTS ${PC_GR_GN3S_LIBDIR}
    PATHS ${GRGN3S_ROOT_USER_DEFINED}/lib
          ${GRGN3S_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
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
