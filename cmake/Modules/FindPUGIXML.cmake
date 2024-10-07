# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Find the pugixml XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
# PUGIXML_INCLUDE_DIR - header location
# PUGIXML_LIBRARIES - library to link against
# PUGIXML_FOUND - true if pugixml was found.
#
# Provides the following imported target:
# Pugixml::pugixml
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_PUGIXML pugixml QUIET)

if(NOT PUGIXML_ROOT)
    set(PUGIXML_ROOT_USER_DEFINED /usr)
else()
    set(PUGIXML_ROOT_USER_DEFINED ${PUGIXML_ROOT})
endif()
if(DEFINED ENV{PUGIXML_ROOT})
    set(PUGIXML_ROOT_USER_DEFINED
        ${PUGIXML_ROOT_USER_DEFINED}
        $ENV{PUGIXML_ROOT}
    )
endif()
if(PUGIXML_HOME)
    set(PUGIXML_ROOT_USER_DEFINED
        ${PUGIXML_ROOT_USER_DEFINED}
        ${PUGIXML_HOME}
    )
endif()

find_path(PUGIXML_INCLUDE_DIR
    NAMES pugixml.hpp
    HINTS ${PC_PUGIXML_INCLUDEDIR}
    PATHS ${PUGIXML_ROOT_USER_DEFINED}/include
          ${PUGIXML_ROOT_USER_DEFINED}/include/pugixml-${PC_PUGIXML_VERSION}
          ${PUGIXML_ROOT_USER_DEFINED}/include/pugixml-1.9
          /usr/include
          /usr/local/include
          /usr/local/include/pugixml-${PC_PUGIXML_VERSION}
          /usr/local/include/pugixml-1.9
          /opt/local/include
)

find_library(PUGIXML_LIBRARY
    NAMES pugixml
    HINTS ${PC_PUGIXML_LIBDIR}
    PATHS ${PUGIXML_ROOT_USER_DEFINED}/lib
          ${PUGIXML_ROOT_USER_DEFINED}/lib64
          ${PUGIXML_ROOT_USER_DEFINED}/lib/pugixml-${PC_PUGIXML_VERSION}
          ${PUGIXML_ROOT_USER_DEFINED}/lib64/pugixml-${PC_PUGIXML_VERSION}
          ${PUGIXML_ROOT_USER_DEFINED}}/lib/pugixml-1.9
          ${PUGIXML_ROOT_USER_DEFINED}/lib64/pugixml-1.9
          ${GNSSSDR_LIB_PATHS}
          /usr/local/lib/pugixml-${PC_PUGIXML_VERSION}
          /usr/local/lib/pugixml-1.9
)

# Support the REQUIRED and QUIET arguments, and set PUGIXML_FOUND if found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PUGIXML DEFAULT_MSG PUGIXML_LIBRARY
        PUGIXML_INCLUDE_DIR)

if(PUGIXML_FOUND)
    set(PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
    if(NOT PUGIXML_FIND_QUIETLY)
        message(STATUS "PugiXML include = ${PUGIXML_INCLUDE_DIR}")
        message(STATUS "PugiXML library = ${PUGIXML_LIBRARY}")
    endif()
    if(PC_PUGIXML_VERSION)
        set(PUGIXML_VERSION ${PC_PUGIXML_VERSION})
    endif()
else()
    message(STATUS "PugiXML not found.")
endif()

set_package_properties(PUGIXML PROPERTIES
    URL "https://pugixml.org/"
)

if(PUGIXML_FOUND AND PUGIXML_VERSION)
    set_package_properties(PUGIXML PROPERTIES
        DESCRIPTION "Light-weight, simple and fast XML parser for C++ (found: v${PUGIXML_VERSION})"
    )
else()
    set_package_properties(PUGIXML PROPERTIES
        DESCRIPTION "Light-weight, simple and fast XML parser for C++"
    )
endif()

mark_as_advanced(PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)

if(PUGIXML_FOUND AND NOT TARGET Pugixml::pugixml)
    add_library(Pugixml::pugixml SHARED IMPORTED)
    set_target_properties(Pugixml::pugixml PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${PUGIXML_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PUGIXML_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${PUGIXML_LIBRARY}"
    )
endif()
