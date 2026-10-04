# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Tries to find libbladeRF.
#
# Usage of this module as follows:
#
# find_package(LIBBLADERF)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# LIBBLADERF_ROOT Set this variable to the root installation of
# libbladeRF if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# LIBBLADERF_FOUND System has libbladeRF libs/headers
# LIBBLADERF_LIBRARIES The libbladeRF libraries (bladeRF)
# LIBBLADERF_INCLUDE_DIRS The location of libbladeRF headers
# LIBBLADERF_VERSION The detected libbladeRF version
#
# Provides the following imported target:
# Bladerf::bladerf
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

pkg_check_modules(PC_LIBBLADERF libbladeRF)

if(NOT LIBBLADERF_ROOT)
    set(LIBBLADERF_ROOT_USER_DEFINED /usr)
else()
    set(LIBBLADERF_ROOT_USER_DEFINED ${LIBBLADERF_ROOT})
endif()
if(DEFINED ENV{LIBBLADERF_ROOT})
    set(LIBBLADERF_ROOT_USER_DEFINED
        ${LIBBLADERF_ROOT_USER_DEFINED}
        $ENV{LIBBLADERF_ROOT}
    )
endif()
set(LIBBLADERF_ROOT_USER_DEFINED
    ${LIBBLADERF_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(LIBBLADERF_INCLUDE_DIRS
    NAMES libbladeRF.h
    HINTS ${PC_LIBBLADERF_INCLUDEDIR}
    PATHS ${LIBBLADERF_ROOT_USER_DEFINED}/include
          ${GNSSSDR_INCLUDE_PATHS}
)

find_library(LIBBLADERF_LIBRARIES
    NAMES bladeRF
    HINTS ${PC_LIBBLADERF_LIBDIR}
    PATHS ${LIBBLADERF_ROOT_USER_DEFINED}/lib
          ${LIBBLADERF_ROOT_USER_DEFINED}/lib64
          ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBBLADERF DEFAULT_MSG LIBBLADERF_LIBRARIES LIBBLADERF_INCLUDE_DIRS)

if(PC_LIBBLADERF_VERSION)
    set(LIBBLADERF_VERSION ${PC_LIBBLADERF_VERSION})
elseif(LIBBLADERF_INCLUDE_DIRS)
    set(_LIBBLADERF_VERSION_HEADER "${LIBBLADERF_INCLUDE_DIRS}/libbladeRF.h")
    if(EXISTS "${_LIBBLADERF_VERSION_HEADER}")
        file(STRINGS "${_LIBBLADERF_VERSION_HEADER}"
            _LIBBLADERF_API_VERSION_LINE
            REGEX "^#[ \t]*define[ \t]+LIBBLADERF_API_VERSION[ \t]+\\(?0[xX][0-9A-Fa-f]+\\)?"
        )
        if(_LIBBLADERF_API_VERSION_LINE MATCHES "0[xX]([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])([0-9A-Fa-f])[0-9A-Fa-f][0-9A-Fa-f]")
            # LIBBLADERF_API_VERSION encodes major, minor, patch, and reserved
            # components in one byte each, starting with the most significant.
            set(_LIBBLADERF_VERSION_HEX_DIGITS "0123456789ABCDEF")
            foreach(_LIBBLADERF_VERSION_COMPONENT MAJOR MINOR PATCH)
                if(_LIBBLADERF_VERSION_COMPONENT STREQUAL "MAJOR")
                    set(_LIBBLADERF_VERSION_HIGH_NIBBLE "${CMAKE_MATCH_1}")
                    set(_LIBBLADERF_VERSION_LOW_NIBBLE "${CMAKE_MATCH_2}")
                elseif(_LIBBLADERF_VERSION_COMPONENT STREQUAL "MINOR")
                    set(_LIBBLADERF_VERSION_HIGH_NIBBLE "${CMAKE_MATCH_3}")
                    set(_LIBBLADERF_VERSION_LOW_NIBBLE "${CMAKE_MATCH_4}")
                else()
                    set(_LIBBLADERF_VERSION_HIGH_NIBBLE "${CMAKE_MATCH_5}")
                    set(_LIBBLADERF_VERSION_LOW_NIBBLE "${CMAKE_MATCH_6}")
                endif()
                string(TOUPPER "${_LIBBLADERF_VERSION_HIGH_NIBBLE}"
                    _LIBBLADERF_VERSION_HIGH_NIBBLE
                )
                string(TOUPPER "${_LIBBLADERF_VERSION_LOW_NIBBLE}"
                    _LIBBLADERF_VERSION_LOW_NIBBLE
                )
                string(FIND "${_LIBBLADERF_VERSION_HEX_DIGITS}"
                    "${_LIBBLADERF_VERSION_HIGH_NIBBLE}"
                    _LIBBLADERF_VERSION_HIGH_NIBBLE
                )
                string(FIND "${_LIBBLADERF_VERSION_HEX_DIGITS}"
                    "${_LIBBLADERF_VERSION_LOW_NIBBLE}"
                    _LIBBLADERF_VERSION_LOW_NIBBLE
                )
                math(EXPR _LIBBLADERF_VERSION_${_LIBBLADERF_VERSION_COMPONENT}
                    "${_LIBBLADERF_VERSION_HIGH_NIBBLE} * 16 + ${_LIBBLADERF_VERSION_LOW_NIBBLE}"
                )
            endforeach()
            set(LIBBLADERF_VERSION
                "${_LIBBLADERF_VERSION_MAJOR}.${_LIBBLADERF_VERSION_MINOR}.${_LIBBLADERF_VERSION_PATCH}"
            )
        endif()
    endif()
endif()

if(LIBBLADERF_FOUND AND DEFINED GNSSSDR_LIBBLADERF_MIN_VERSION)
    if(NOT LIBBLADERF_VERSION)
        message(STATUS "libbladeRF was found, but its version could not be determined. GNSS-SDR requires at least v${GNSSSDR_LIBBLADERF_MIN_VERSION}")
        set(LIBBLADERF_FOUND FALSE)
        unset(LIBBLADERF_LIBRARIES CACHE)
        unset(LIBBLADERF_INCLUDE_DIRS CACHE)
    elseif(LIBBLADERF_VERSION VERSION_LESS GNSSSDR_LIBBLADERF_MIN_VERSION)
        message(STATUS "libbladeRF v${LIBBLADERF_VERSION} was found, but GNSS-SDR requires at least v${GNSSSDR_LIBBLADERF_MIN_VERSION}. Please upgrade libbladeRF: https://github.com/Nuand/bladeRF")
        set(LIBBLADERF_FOUND FALSE)
        unset(LIBBLADERF_LIBRARIES CACHE)
        unset(LIBBLADERF_INCLUDE_DIRS CACHE)
    endif()
endif()

if(LIBBLADERF_FOUND AND LIBBLADERF_VERSION)
    set_package_properties(LIBBLADERF PROPERTIES
        DESCRIPTION "Nuand bladeRF library (found: v${LIBBLADERF_VERSION})"
    )
else()
    set_package_properties(LIBBLADERF PROPERTIES
        DESCRIPTION "Nuand bladeRF library"
    )
endif()

set_package_properties(LIBBLADERF PROPERTIES
    URL "https://github.com/Nuand/bladeRF"
)

if(LIBBLADERF_FOUND AND NOT TARGET Bladerf::bladerf)
    add_library(Bladerf::bladerf SHARED IMPORTED)
    set_target_properties(Bladerf::bladerf PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBBLADERF_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBBLADERF_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBBLADERF_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBBLADERF_LIBRARIES LIBBLADERF_INCLUDE_DIRS)
