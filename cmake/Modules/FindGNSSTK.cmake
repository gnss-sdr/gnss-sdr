# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# - Find gnsstk library
# Find the native gnsstk includes and library
# This module defines
#  GNSSTK_INCLUDE_DIR, where to find Rinex3ObsBase.hpp, etc.
#  GNSSTK_FOUND, If false, do not try to use GNSSTK.
#  GNSSTK_LIBRARY, where to find the GNSSTK library.
#
# Provides the following imported target:
# Gnsstk::gnsstk
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSTK_ROOT)
    set(GNSSTK_ROOT_USER_DEFINED /usr/local)
else()
    set(GNSSTK_ROOT_USER_DEFINED ${GNSSTK_ROOT})
endif()
if(DEFINED ENV{GNSSTK_ROOT})
    set(GNSSTK_ROOT_USER_DEFINED
        ${GNSSTK_ROOT_USER_DEFINED}
        $ENV{GNSSTK_ROOT}
    )
endif()

find_path(GNSSTK_INCLUDE_DIR gnsstk/Rinex3ObsBase.hpp
    PATHS ${GNSSTK_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

set(GNSSTK_NAMES gnsstk libgnsstk)

include(GNUInstallDirs)

find_library(GNSSTK_LIBRARY NAMES ${GNSSTK_NAMES}
    PATHS ${GNSSTK_ROOT_USER_DEFINED}/lib
          ${GNSSTK_ROOT_USER_DEFINED}/${CMAKE_INSTALL_LIBDIR}
          /usr/local/lib
          /usr/${CMAKE_INSTALL_LIBDIR}
          /usr/local/${CMAKE_INSTALL_LIBDIR}
          /opt/local/lib
)

# handle the QUIET and REQUIRED arguments and set GNSSTK_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNSSTK DEFAULT_MSG GNSSTK_LIBRARY GNSSTK_INCLUDE_DIR)

if(GNSSTK_FOUND)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    if(EXISTS ${CMAKE_INSTALL_FULL_DATADIR}/cmake/GNSSTK/GNSSTKConfigVersion.cmake)
        include(${CMAKE_INSTALL_FULL_DATADIR}/cmake/GNSSTK/GNSSTKConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(GNSSTK_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

if(GNSSTK_FOUND AND GNSSTK_VERSION)
    set_package_properties(GNSSTK PROPERTIES
        DESCRIPTION "Library and suite of applications for satellite navigation (found: v${GNSSTK_VERSION})"
    )
else()
    set_package_properties(GNSSTK PROPERTIES
        DESCRIPTION "Library and suite of applications for satellite navigation"
    )
endif()

#if(GNSSTK_FOUND AND NOT EXISTS ${GNSSTK_INCLUDE_DIR}/gnsstk/SatelliteSystem.hpp)
#    set(GNSSTK_OLDER_THAN_8 TRUE)
#endif()

set_package_properties(GNSSTK PROPERTIES
    URL "https://github.com/SGL-UT/gnsstk/"
)

if(GNSSTK_FOUND AND NOT ENABLE_OWN_GNSSTK AND NOT TARGET Gnsstk::gnsstk)
    add_library(Gnsstk::gnsstk SHARED IMPORTED)
    set_target_properties(Gnsstk::gnsstk PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GNSSTK_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${GNSSTK_INCLUDE_DIR};${GNSSTK_INCLUDE_DIR}/gnsstk"
        INTERFACE_LINK_LIBRARIES "${GNSSTK_LIBRARY}"
    )
endif()

mark_as_advanced(GNSSTK_LIBRARY GNSSTK_INCLUDE_DIR)
