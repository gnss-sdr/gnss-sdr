# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_TELEORBIT teleorbit QUIET)

find_path(TELEORBIT_INCLUDE_DIRS
    NAMES teleorbit/api.h
    HINTS $ENV{TELEORBIT_DIR}/include
          ${PC_TELEORBIT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
          /opt/local/include
          ${TELEORBIT_ROOT}/include
          $ENV{TELEORBIT_ROOT}/include
)

find_library(TELEORBIT_LIBRARIES
    NAMES gnuradio-teleorbit
    HINTS $ENV{TELEORBIT_DIR}/lib
          ${PC_TELEORBIT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          /opt/local/lib
          ${TELEORBIT_ROOT}/lib
          $ENV{TELEORBIT_ROOT}/lib
          ${TELEORBIT_ROOT}/lib64
          $ENV{TELEORBIT_ROOT}/lib64
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
