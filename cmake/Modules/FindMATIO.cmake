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

# FindMATIO
#
# Try to find MATIO library
#
# Once done this will define:
#
#  MATIO_FOUND - True if MATIO found.
#  MATIO_LIBRARIES - MATIO libraries.
#  MATIO_INCLUDE_DIRS - where to find matio.h, etc..
#  MATIO_VERSION_STRING - version number as a string (e.g.: "1.3.4")
#
#=============================================================================
# Copyright 2015 Avtech Scientific <http://avtechscientific.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the names of Kitware, Inc., the Insight Software Consortium,
#   nor the names of their contributors may be used to endorse or promote
#   products derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================
#

# Look for the header file.
find_path(MATIO_INCLUDE_DIR NAMES matio.h DOC "The MATIO include directory")

# Look for the library.
find_library(MATIO_LIBRARY NAMES matio DOC "The MATIO library")

if(MATIO_INCLUDE_DIR)
    # ---------------------------------------------------
    #  Extract version information from MATIO
    # ---------------------------------------------------

    # If the file is missing, set all values to 0
    set(MATIO_MAJOR_VERSION 0)
    set(MATIO_MINOR_VERSION 0)
    set(MATIO_RELEASE_LEVEL 0)

    # new versions of MATIO have `matio_pubconf.h`
    if(EXISTS ${MATIO_INCLUDE_DIR}/matio_pubconf.h)
        set(MATIO_CONFIG_FILE "matio_pubconf.h")
    else()
        set(MATIO_CONFIG_FILE "matioConfig.h")
    endif()

    if(MATIO_CONFIG_FILE)

        # Read and parse MATIO config header file for version number
        file(STRINGS "${MATIO_INCLUDE_DIR}/${MATIO_CONFIG_FILE}" _matio_HEADER_CONTENTS REGEX "#define MATIO_((MAJOR|MINOR)_VERSION)|(RELEASE_LEVEL) ")

        foreach(line ${_matio_HEADER_CONTENTS})
            if(line MATCHES "#define ([A-Z_]+) ([0-9]+)")
                set("${CMAKE_MATCH_1}" "${CMAKE_MATCH_2}")
            endif()
        endforeach()

        unset(_matio_HEADER_CONTENTS)
    endif()

    set(MATIO_VERSION_STRING "${MATIO_MAJOR_VERSION}.${MATIO_MINOR_VERSION}.${MATIO_RELEASE_LEVEL}")
endif()

mark_as_advanced(MATIO_INCLUDE_DIR MATIO_LIBRARY)

# handle the QUIETLY and REQUIRED arguments and set MATIO_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATIO REQUIRED_VARS MATIO_LIBRARY MATIO_INCLUDE_DIR VERSION_VAR MATIO_VERSION_STRING)

if(MATIO_FOUND)
  set(MATIO_LIBRARIES ${MATIO_LIBRARY})
  set(MATIO_INCLUDE_DIRS ${MATIO_INCLUDE_DIR})
else()
  set(MATIO_LIBRARIES)
  set(MATIO_INCLUDE_DIRS)
endif()
