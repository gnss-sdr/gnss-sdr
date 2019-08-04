# Copyright (C) 2011-2019 (see AUTHORS file for a list of contributors)
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

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_PUGIXML pugixml QUIET)

find_path(PUGIXML_INCLUDE_DIR
    NAMES pugixml.hpp
    PATHS ${PUGIXML_HOME}/include
          /usr/include
          /usr/local/include
          /usr/local/include/pugixml-1.9
          /usr/local/include/pugixml-${PC_PUGIXML_VERSION}
          /opt/local/include
          ${PUGIXML_ROOT}/include
          $ENV{PUGIXML_ROOT}/include
          ${PUGIXML_ROOT}/include/pugixml-1.9
          $ENV{PUGIXML_ROOT}/include/pugixml-1.9
          ${PUGIXML_ROOT}/include/pugixml-${PC_PUGIXML_VERSION}
          $ENV{PUGIXML_ROOT}/include/pugixml-${PC_PUGIXML_VERSION}
          ${PC_PUGIXML_INCLUDEDIR}
)

find_library(PUGIXML_LIBRARY
    NAMES pugixml
    PATHS ${PUGIXML_HOME}/lib
          /usr/lib/x86_64-linux-gnu
          /usr/lib/aarch64-linux-gnu
          /usr/lib/arm-linux-gnueabi
          /usr/lib/arm-linux-gnueabihf
          /usr/lib/i386-linux-gnu
          /usr/lib/mips-linux-gnu
          /usr/lib/mips64el-linux-gnuabi64
          /usr/lib/mipsel-linux-gnu
          /usr/lib/powerpc64le-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/local/lib
          /usr/local/lib/pugixml-1.9
          /usr/local/lib/pugixml-${PC_PUGIXML_VERSION}
          /opt/local/lib
          /usr/lib
          /usr/lib64
          /usr/local/lib64
          ${PUGIXML_ROOT}/lib
          $ENV{PUGIXML_ROOT}/lib
          ${PUGIXML_ROOT}/lib64
          $ENV{PUGIXML_ROOT}/lib64
          ${PUGIXML_ROOT}/lib/pugixml-1.9
          $ENV{PUGIXML_ROOT}/lib/pugixml-1.9
          ${PUGIXML_ROOT}/lib64/pugixml-1.9
          $ENV{PUGIXML_ROOT}/lib64/pugixml-1.9
          ${PUGIXML_ROOT}/lib/pugixml-${PC_PUGIXML_VERSION}
          $ENV{PUGIXML_ROOT}/lib/pugixml-${PC_PUGIXML_VERSION}
          ${PUGIXML_ROOT}/lib64/pugixml-${PC_PUGIXML_VERSION}
          $ENV{PUGIXML_ROOT}/lib64/pugixml-${PC_PUGIXML_VERSION}
          ${PC_PUGIXML_LIBDIR}
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
