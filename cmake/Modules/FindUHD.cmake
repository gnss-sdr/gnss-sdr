# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
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

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_UHD uhd)

if(NOT UHD_ROOT)
    set(UHD_ROOT_USER_PROVIDED /usr/local)
else()
    set(UHD_ROOT_USER_PROVIDED ${UHD_ROOT})
endif()
if(GNURADIO_INSTALL_PREFIX)
    set(UHD_ROOT_USER_PROVIDED
        ${UHD_ROOT_USER_PROVIDED}
        ${GNURADIO_INSTALL_PREFIX}
    )
endif()
if(DEFINED ENV{UHD_ROOT})
    set(UHD_ROOT_USER_PROVIDED
        ${UHD_ROOT_USER_PROVIDED}
        $ENV{UHD_ROOT}
    )
endif()
if(DEFINED ENV{UHD_DIR})
    set(UHD_ROOT_USER_PROVIDED
        ${UHD_ROOT_USER_PROVIDED}
        $ENV{UHD_DIR}
    )
endif()

find_path(UHD_INCLUDE_DIRS
    NAMES uhd/config.hpp
    HINTS ${PC_UHD_INCLUDEDIR}
    PATHS ${UHD_ROOT_USER_PROVIDED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(UHD_LIBRARIES
    NAMES uhd
    HINTS ${PC_UHD_LIBDIR}
    PATHS ${UHD_ROOT_USER_PROVIDED}/lib
          ${UHD_ROOT_USER_PROVIDED}/lib64
          ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(UHD DEFAULT_MSG UHD_LIBRARIES UHD_INCLUDE_DIRS)

if(PC_UHD_VERSION)
    set(UHD_VERSION ${PC_UHD_VERSION})
endif()
if(NOT PC_UHD_VERSION)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET UHD_LIBRARIES 0 FIRST_DIR)
    get_filename_component(UHD_LIBRARIES_DIR ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${UHD_LIBRARIES_DIR}/cmake/uhd/UHDConfigVersion.cmake)
        include(${UHD_LIBRARIES_DIR}/cmake/uhd/UHDConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(UHD_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
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
    add_library(Uhd::uhd SHARED IMPORTED)
    set_target_properties(Uhd::uhd PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${UHD_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${UHD_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${UHD_LIBRARIES}"
    )
endif()

mark_as_advanced(UHD_LIBRARIES UHD_INCLUDE_DIRS)
