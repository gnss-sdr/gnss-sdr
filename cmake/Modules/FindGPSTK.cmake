# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# - Find gpstk library
# Find the native gpstk includes and library
# This module defines
#  GPSTK_INCLUDE_DIR, where to find Rinex3ObsBase.hpp, etc.
#  GPSTK_FOUND, If false, do not try to use GPSTK.
#  GPSTK_LIBRARY, where to find the GPSTK library.
#
# Provides the following imported target:
# Gpstk::gpstk
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GPSTK_ROOT)
    set(GPSTK_ROOT_USER_DEFINED /usr/local)
else()
    set(GPSTK_ROOT_USER_DEFINED ${GPSTK_ROOT})
endif()
if(DEFINED ENV{GPSTK_ROOT})
    set(GPSTK_ROOT_USER_DEFINED
        ${GPSTK_ROOT_USER_DEFINED}
        $ENV{GPSTK_ROOT}
    )
endif()

find_path(GPSTK_INCLUDE_DIR gpstk/Rinex3ObsBase.hpp
    PATHS ${GPSTK_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

set(GPSTK_NAMES gpstk libgpstk)

include(GNUInstallDirs)

find_library(GPSTK_LIBRARY NAMES ${GPSTK_NAMES}
    PATHS ${GPSTK_ROOT_USER_DEFINED}/lib
          ${GPSTK_ROOT_USER_DEFINED}/${CMAKE_INSTALL_LIBDIR}
          /usr/local/lib
          /usr/${CMAKE_INSTALL_LIBDIR}
          /usr/local/${CMAKE_INSTALL_LIBDIR}
          /opt/local/lib
)

# handle the QUIET and REQUIRED arguments and set GPSTK_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GPSTK DEFAULT_MSG GPSTK_LIBRARY GPSTK_INCLUDE_DIR)

if(GPSTK_FOUND)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    if(EXISTS ${CMAKE_INSTALL_FULL_DATADIR}/cmake/GPSTK/GPSTKConfigVersion.cmake)
        include(${CMAKE_INSTALL_FULL_DATADIR}/cmake/GPSTK/GPSTKConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(GPSTK_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

if(GPSTK_FOUND AND GPSTK_VERSION)
    set_package_properties(GPSTK PROPERTIES
        DESCRIPTION "Library and suite of applications for satellite navigation (found: v${GPSTK_VERSION})"
    )
else()
    set_package_properties(GPSTK PROPERTIES
        DESCRIPTION "Library and suite of applications for satellite navigation"
    )
endif()

if(GPSTK_FOUND AND NOT EXISTS ${GPSTK_INCLUDE_DIR}/gpstk/SatelliteSystem.hpp)
    set(GPSTK_OLDER_THAN_8 TRUE)
endif()

set_package_properties(GPSTK PROPERTIES
    URL "https://github.com/SGL-UT/GPSTk"
)

if(GPSTK_FOUND AND NOT ENABLE_OWN_GPSTK AND NOT TARGET Gpstk::gpstk)
    add_library(Gpstk::gpstk SHARED IMPORTED)
    set_target_properties(Gpstk::gpstk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPSTK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPSTK_INCLUDE_DIR};${GPSTK_INCLUDE_DIR}/gpstk"
        INTERFACE_LINK_LIBRARIES "${GPSTK_LIBRARY}"
    )
endif()

mark_as_advanced(GPSTK_LIBRARY GPSTK_INCLUDE_DIR)
