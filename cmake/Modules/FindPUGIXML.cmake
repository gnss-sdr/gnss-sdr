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

# Find the pugixml XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
# PUGIXML_INCLUDE_DIR - header location
# PUGIXML_LIBRARIES - library to link against
# PUGIXML_FOUND - true if pugixml was found.

find_path(PUGIXML_INCLUDE_DIR
    NAMES pugixml.hpp
    PATHS ${PUGIXML_HOME}/include
        /usr/include
        /usr/local/include
        /opt/local/include)

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
        /opt/local/lib
        /usr/lib
        /usr/lib64
        /usr/local/lib64)

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
else()
    message(STATUS "PugiXML not found.")
endif()

mark_as_advanced(PUGIXML_LIBRARY PUGIXML_INCLUDE_DIR)
