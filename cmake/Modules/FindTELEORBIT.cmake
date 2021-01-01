# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Gnuradio::teleorbit
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_TELEORBIT teleorbit QUIET)

if(NOT TELEORBIT_ROOT)
    set(TELEORBIT_ROOT_USER_DEFINED /usr/local)
else()
    set(TELEORBIT_ROOT_USER_DEFINED ${TELEORBIT_ROOT})
endif()
if(DEFINED ENV{TELEORBIT_ROOT})
    set(TELEORBIT_ROOT_USER_DEFINED
        ${TELEORBIT_ROOT_USER_DEFINED}
        $ENV{TELEORBIT_ROOT}
    )
endif()
set(TELEORBIT_ROOT_USER_DEFINED
    ${TELEORBIT_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(TELEORBIT_INCLUDE_DIRS
    NAMES teleorbit/api.h
    HINTS ${PC_TELEORBIT_INCLUDEDIR}
    PATHS ${TELEORBIT_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(TELEORBIT_LIBRARIES
    NAMES gnuradio-teleorbit
    HINTS ${PC_TELEORBIT_LIBDIR}
    PATHS ${TELEORBIT_ROOT_USER_DEFINED}/lib
          ${TELEORBIT_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TELEORBIT DEFAULT_MSG TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)

if(PC_TELEORBIT_VERSION)
    set(TELEORBIT_VERSION ${PC_TELEORBIT_VERSION})
endif()

if(NOT TELEORBIT_VERSION)
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET TELEORBIT_LIBRARIES 0 FIRST_DIR)
    get_filename_component(TELEORBIT_LIBRARIES_DIR ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${TELEORBIT_LIBRARIES_DIR}/cmake/teleorbit/TeleorbitConfigVersion.cmake)
        include(${TELEORBIT_LIBRARIES_DIR}/cmake/teleorbit/TeleorbitConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(TELEORBIT_VERSION ${PACKAGE_VERSION})
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

if(TELEORBIT_FOUND AND TELEORBIT_VERSION)
    set_package_properties(TELEORBIT PROPERTIES
        DESCRIPTION "The Teleorbit's Flexiband GNU Radio block (found: v${TELEORBIT_VERSION})"
    )
else()
    set_package_properties(TELEORBIT PROPERTIES
        DESCRIPTION "The Teleorbit's Flexiband GNU Radio block."
    )
endif()

if(TELEORBIT_FOUND AND NOT TARGET Gnuradio::teleorbit)
    add_library(Gnuradio::teleorbit SHARED IMPORTED)
    set_target_properties(Gnuradio::teleorbit PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${TELEORBIT_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${TELEORBIT_INCLUDE_DIRS};${TELEORBIT_INCLUDE_DIRS}/teleorbit"
        INTERFACE_LINK_LIBRARIES "${TELEORBIT_LIBRARIES}"
    )
endif()

mark_as_advanced(TELEORBIT_LIBRARIES TELEORBIT_INCLUDE_DIRS)
