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

unset(GNSSTK_INCLUDE_DIR CACHE)
unset(GNSSTK_USES_GPSTK_NAMESPACE CACHE)
find_path(GNSSTK_INCLUDE_DIR gnsstk/Rinex3ObsBase.hpp
    PATHS ${GNSSTK_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)
set(GNSSTK_NAMES ${CMAKE_FIND_LIBRARY_PREFIXES}gnsstk${CMAKE_SHARED_LIBRARY_SUFFIX})
if(NOT GNSSTK_INCLUDE_DIR)
    find_path(GNSSTK_INCLUDE_DIR gpstk/Rinex3ObsBase.hpp
        PATHS ${GNSSTK_ROOT_USER_DEFINED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )
    if(GNSSTK_INCLUDE_DIR)
        set(GNSSTK_NAMES gpstk ${CMAKE_FIND_LIBRARY_PREFIXES}gpstk${CMAKE_SHARED_LIBRARY_SUFFIX})
        set(GNSSTK_USES_GPSTK_NAMESPACE TRUE)
    endif()
endif()

include(GNUInstallDirs)

find_library(GNSSTK_LIBRARY NAMES ${GNSSTK_NAMES}
    PATHS ${GNSSTK_ROOT_USER_DEFINED}/lib
          ${GNSSTK_ROOT_USER_DEFINED}/${CMAKE_INSTALL_LIBDIR}
          /usr/local/lib
          /usr/${CMAKE_INSTALL_LIBDIR}
          /usr/local/${CMAKE_INSTALL_LIBDIR}
          /opt/local/lib
)

if(GNSSTK_LIBRARY AND GNSSTK_INCLUDE_DIR)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    if(GNSSTK_USES_GPSTK_NAMESPACE)
        if(EXISTS ${GNSSTK_INCLUDE_DIR}/../share/cmake/GPSTK/GPSTKConfigVersion.cmake)
            include(${GNSSTK_INCLUDE_DIR}/../share/cmake/GPSTK/GPSTKConfigVersion.cmake)
        endif()
    else()
        if(EXISTS ${GNSSTK_INCLUDE_DIR}/../share/cmake/GNSSTK/GNSSTKConfigVersion.cmake)
            include(${GNSSTK_INCLUDE_DIR}/../share/cmake/GNSSTK/GNSSTKConfigVersion.cmake)
        endif()
    endif()
    if(PACKAGE_VERSION)
        set(GNSSTK_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

if(GNSSTK_VERSION)
    if(GNSSTK_VERSION VERSION_LESS "9.0.0")
        set(GNSSTK_OLDER_THAN_9 TRUE)
    endif()
    if(GNSSTK_VERSION VERSION_LESS "13.0.0")
        set(GNSSTK_OLDER_THAN_13 TRUE)
    endif()
endif()

if(EXISTS ${GNSSTK_INCLUDE_DIR}/gnsstk/GPSEphemerisStore.hpp)
    set(GNSSTK_OLDER_THAN_13 TRUE)
endif()

# handle the QUIET and REQUIRED arguments and set GNSSTK_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNSSTK DEFAULT_MSG GNSSTK_LIBRARY GNSSTK_INCLUDE_DIR)

if(GNSSTK_FOUND AND GNSSTK_VERSION)
    set_package_properties(GNSSTK PROPERTIES
        DESCRIPTION "The GNSSTk C++ Library (found: v${GNSSTK_VERSION})"
    )
else()
    set_package_properties(GNSSTK PROPERTIES
        DESCRIPTION "The GNSSTk C++ Library"
    )
endif()

if(GNSSTK_FOUND AND GNSSTK_USES_GPSTK_NAMESPACE AND NOT EXISTS ${GNSSTK_INCLUDE_DIR}/gpstk/SatelliteSystem.hpp)
    set(GNSSTK_OLDER_THAN_8 TRUE)
endif()

set_package_properties(GNSSTK PROPERTIES
    URL "https://github.com/SGL-UT/gnsstk/"
    TYPE OPTIONAL
)

if(GNSSTK_FOUND AND NOT ENABLE_OWN_GNSSTK AND NOT TARGET Gnsstk::gnsstk)
    add_library(Gnsstk::gnsstk SHARED IMPORTED)
    if(GNSSTK_USES_GPSTK_NAMESPACE)
        set_target_properties(Gnsstk::gnsstk PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GNSSTK_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GNSSTK_INCLUDE_DIR};${GNSSTK_INCLUDE_DIR}/gpstk"
            INTERFACE_LINK_LIBRARIES "${GNSSTK_LIBRARY}"
            IMPORTED_IMPLIB "${GNSSTK_LIBRARY}"
        )
    else()
        set_target_properties(Gnsstk::gnsstk PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${GNSSTK_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GNSSTK_INCLUDE_DIR};${GNSSTK_INCLUDE_DIR}/gnsstk"
            INTERFACE_LINK_LIBRARIES "${GNSSTK_LIBRARY}"
            IMPORTED_IMPLIB "${GNSSTK_LIBRARY}"
        )
    endif()
endif()

mark_as_advanced(GNSSTK_LIBRARY
    GNSSTK_INCLUDE_DIR
    GNSSTK_USES_GPSTK_NAMESPACE
    GNSSTK_OLDER_THAN_8
    GNSSTK_OLDER_THAN_9
    GNSSTK_OLDER_THAN_13
)
