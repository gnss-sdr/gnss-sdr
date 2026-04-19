# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Uhd::uhd
#

########################################################################
# Find the library for the USRP Hardware Driver
########################################################################
if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

pkg_check_modules(PC_UHD uhd)

set(UHD_ROOT_USER_PROVIDED "")
if(DEFINED UHD_ROOT AND NOT "${UHD_ROOT}" STREQUAL "")
    list(APPEND UHD_ROOT_USER_PROVIDED "${UHD_ROOT}")
else()
    list(APPEND UHD_ROOT_USER_PROVIDED "/usr/local")
endif()

if(GNURADIO_INSTALL_PREFIX)
    list(APPEND UHD_ROOT_USER_PROVIDED
        "${GNURADIO_INSTALL_PREFIX}"
    )
endif()

if(DEFINED ENV{UHD_ROOT})
    list(APPEND UHD_ROOT_USER_PROVIDED
        "$ENV{UHD_ROOT}"
    )
endif()

if(DEFINED ENV{UHD_DIR})
    list(APPEND UHD_ROOT_USER_PROVIDED
        "$ENV{UHD_DIR}"
    )
endif()

find_path(UHD_INCLUDE_DIRS
    NAMES uhd/config.hpp
    HINTS ${PC_UHD_INCLUDEDIR}
    PATHS
        ${UHD_ROOT_USER_PROVIDED}/include
        ${GNSSSDR_INCLUDE_PATHS}
)

find_library(UHD_LIBRARIES
    NAMES uhd
    HINTS ${PC_UHD_LIBDIR}
    PATHS
        ${UHD_ROOT_USER_PROVIDED}/lib
        ${UHD_ROOT_USER_PROVIDED}/lib64
        ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UHD DEFAULT_MSG
    UHD_LIBRARIES
    UHD_INCLUDE_DIRS
)

if(PC_UHD_VERSION)
    set(UHD_VERSION "${PC_UHD_VERSION}")
else()
    if(DEFINED PACKAGE_VERSION)
        set(_UHD_OLD_PACKAGE_VERSION "${PACKAGE_VERSION}")
        set(_UHD_HAD_PACKAGE_VERSION TRUE)
    else()
        unset(_UHD_OLD_PACKAGE_VERSION)
        set(_UHD_HAD_PACKAGE_VERSION FALSE)
    endif()
    unset(PACKAGE_VERSION)

    unset(_UHD_VERSION_CMAKE_FILES)
    unset(_UHD_VERSION_HEADER)
    unset(_UHD_VERSION_FROM_HEADER)

    if(UHD_LIBRARIES)
        get_filename_component(UHD_LIBRARIES_DIR "${UHD_LIBRARIES}" DIRECTORY)
        get_filename_component(UHD_PREFIX_DIR "${UHD_LIBRARIES_DIR}" DIRECTORY)

        set(_UHD_VERSION_CMAKE_FILES
            "${UHD_LIBRARIES_DIR}/cmake/uhd/UHDConfigVersion.cmake"
            "${UHD_LIBRARIES_DIR}/cmake/UHD/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/lib/cmake/uhd/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/lib/cmake/UHD/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/lib64/cmake/uhd/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/lib64/cmake/UHD/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/share/uhd/cmake/UHDConfigVersion.cmake"
            "${UHD_PREFIX_DIR}/share/UHD/cmake/UHDConfigVersion.cmake"
        )

        foreach(_uhd_version_file IN LISTS _UHD_VERSION_CMAKE_FILES)
            if(EXISTS "${_uhd_version_file}")
                include("${_uhd_version_file}")
                if(PACKAGE_VERSION)
                    break()
                endif()
            endif()
        endforeach()
    endif()

    if(PACKAGE_VERSION)
        set(UHD_VERSION "${PACKAGE_VERSION}")
    elseif(UHD_INCLUDE_DIRS)
        set(_UHD_VERSION_HEADER "${UHD_INCLUDE_DIRS}/uhd/version.hpp")

        if(EXISTS "${_UHD_VERSION_HEADER}")
            file(STRINGS "${_UHD_VERSION_HEADER}"
                _uhd_version_major_line
                REGEX "^#define[ \t]+UHD_VERSION_MAJOR[ \t]+[0-9]+"
            )
            file(STRINGS "${_UHD_VERSION_HEADER}"
                _uhd_version_api_line
                REGEX "^#define[ \t]+UHD_VERSION_API[ \t]+[0-9]+"
            )
            file(STRINGS "${_UHD_VERSION_HEADER}"
                _uhd_version_abi_line
                REGEX "^#define[ \t]+UHD_VERSION_ABI[ \t]+[0-9]+"
            )
            file(STRINGS "${_UHD_VERSION_HEADER}"
                _uhd_version_abi_string_line
                REGEX "^#define[ \t]+UHD_VERSION_ABI_STRING[ \t]+\"[^\"]+\""
            )

            if(_uhd_version_abi_string_line)
                string(REGEX REPLACE
                    "^#define[ \t]+UHD_VERSION_ABI_STRING[ \t]+\"([^\"]+)\"$"
                    "\\1"
                    _uhd_version_abi_string
                    "${_uhd_version_abi_string_line}"
                )
                set(_UHD_VERSION_FROM_HEADER "${_uhd_version_abi_string}")
            elseif(_uhd_version_major_line AND _uhd_version_api_line
                    AND _uhd_version_abi_line)
                string(REGEX REPLACE
                    "^#define[ \t]+UHD_VERSION_MAJOR[ \t]+([0-9]+)$"
                    "\\1"
                    _uhd_version_major
                    "${_uhd_version_major_line}"
                )
                string(REGEX REPLACE
                    "^#define[ \t]+UHD_VERSION_API[ \t]+([0-9]+)$"
                    "\\1"
                    _uhd_version_api
                    "${_uhd_version_api_line}"
                )
                string(REGEX REPLACE
                    "^#define[ \t]+UHD_VERSION_ABI[ \t]+([0-9]+)$"
                    "\\1"
                    _uhd_version_abi
                    "${_uhd_version_abi_line}"
                )
                set(_UHD_VERSION_FROM_HEADER
                    "${_uhd_version_major}.${_uhd_version_api}.${_uhd_version_abi}")
            endif()
        endif()

        if(_UHD_VERSION_FROM_HEADER)
            set(UHD_VERSION "${_UHD_VERSION_FROM_HEADER}")
        endif()
    endif()

    if(_UHD_HAD_PACKAGE_VERSION)
        set(PACKAGE_VERSION "${_UHD_OLD_PACKAGE_VERSION}")
    else()
        unset(PACKAGE_VERSION)
    endif()
endif()

set_package_properties(UHD PROPERTIES
    URL "https://www.ettus.com/sdr-software/uhd-usrp-hardware-driver/"
)

if(UHD_FOUND AND UHD_VERSION)
    set_package_properties(UHD PROPERTIES
        DESCRIPTION "USRP Hardware Driver (found: v${UHD_VERSION})"
    )
else()
    set_package_properties(UHD PROPERTIES
        DESCRIPTION "USRP Hardware Driver"
    )
endif()

if(UHD_FOUND AND NOT TARGET Uhd::uhd)
    add_library(Uhd::uhd UNKNOWN IMPORTED)
    set_target_properties(Uhd::uhd PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${UHD_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${UHD_INCLUDE_DIRS}"
    )
endif()

mark_as_advanced(UHD_LIBRARIES UHD_INCLUDE_DIRS)
