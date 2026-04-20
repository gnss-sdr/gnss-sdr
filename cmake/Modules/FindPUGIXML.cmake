# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
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

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(PC_PUGIXML QUIET pugixml)

set(PUGIXML_ROOT_USER_DEFINED "")
if(DEFINED PUGIXML_ROOT AND NOT "${PUGIXML_ROOT}" STREQUAL "")
    list(APPEND PUGIXML_ROOT_USER_DEFINED "${PUGIXML_ROOT}")
else()
    list(APPEND PUGIXML_ROOT_USER_DEFINED "/usr")
endif()

if(DEFINED ENV{PUGIXML_ROOT})
    list(APPEND PUGIXML_ROOT_USER_DEFINED "$ENV{PUGIXML_ROOT}")
endif()

if(DEFINED PUGIXML_HOME AND NOT "${PUGIXML_HOME}" STREQUAL "")
    list(APPEND PUGIXML_ROOT_USER_DEFINED "${PUGIXML_HOME}")
endif()

find_path(PUGIXML_INCLUDE_DIR
    NAMES pugixml.hpp
    HINTS ${PC_PUGIXML_INCLUDEDIR}
    PATHS
        ${PUGIXML_ROOT_USER_DEFINED}/include
        ${GNSSSDR_INCLUDE_PATHS}
    PATH_SUFFIXES
        pugixml-${PC_PUGIXML_VERSION}
        pugixml-1.9
)

find_library(PUGIXML_LIBRARY
    NAMES pugixml
    HINTS ${PC_PUGIXML_LIBDIR}
    PATHS
        ${PUGIXML_ROOT_USER_DEFINED}/lib
        ${PUGIXML_ROOT_USER_DEFINED}/lib64
        ${GNSSSDR_LIB_PATHS}
    PATH_SUFFIXES
        pugixml-${PC_PUGIXML_VERSION}
        pugixml-1.9
)

# Support the REQUIRED and QUIET arguments, and set PUGIXML_FOUND if found.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PUGIXML DEFAULT_MSG
    PUGIXML_LIBRARY
    PUGIXML_INCLUDE_DIR
)

if(PUGIXML_FOUND)
    set(PUGIXML_LIBRARIES "${PUGIXML_LIBRARY}")

    if(NOT PUGIXML_FIND_QUIETLY)
        message(STATUS "PugiXML include = ${PUGIXML_INCLUDE_DIR}")
        message(STATUS "PugiXML library = ${PUGIXML_LIBRARY}")
    endif()

    if(PC_PUGIXML_VERSION)
        set(PUGIXML_VERSION "${PC_PUGIXML_VERSION}")
    endif()
endif()

set_package_properties(PUGIXML PROPERTIES
    URL "https://pugixml.org/"
)

if(PUGIXML_FOUND AND PUGIXML_VERSION)
    set_package_properties(PUGIXML PROPERTIES
        DESCRIPTION
        "Light-weight, simple and fast XML parser for C++ (found: v${PUGIXML_VERSION})"
    )
else()
    set_package_properties(PUGIXML PROPERTIES
        DESCRIPTION "Light-weight, simple and fast XML parser for C++"
    )
endif()

mark_as_advanced(PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)

if(PUGIXML_FOUND AND NOT TARGET Pugixml::pugixml)
    add_library(Pugixml::pugixml UNKNOWN IMPORTED)
    set_target_properties(Pugixml::pugixml PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${PUGIXML_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PUGIXML_INCLUDE_DIR}"
    )
endif()
