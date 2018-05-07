# Copyright (C) 2014-2018 (see AUTHORS file for a list of contributors)
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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.

if(DEFINED __INCLUDED_VOLK_VERSION_CMAKE)
    return()
endif()
set(__INCLUDED_VOLK_VERSION_CMAKE TRUE)

#eventually, replace version.sh and fill in the variables below
set(MAJOR_VERSION ${VERSION_INFO_MAJOR_VERSION})
set(MINOR_VERSION ${VERSION_INFO_MINOR_VERSION})
set(MAINT_VERSION ${VERSION_INFO_MAINT_VERSION})

########################################################################
# Extract the version string from git describe.
########################################################################
find_package(Git)

if(GIT_FOUND AND EXISTS ${PROJECT_SOURCE_DIR}/.git)
    message(STATUS "Extracting version information from git describe...")
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --always --abbrev=8 --long
        OUTPUT_VARIABLE GIT_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    )
else()
  if(NOT VOLK_GIT_COUNT)
    set(VOLK_GIT_COUNT "0")
  endif()

  if(NOT VOLK_GIT_HASH)
    set(VOLK_GIT_HASH "unknown")
  endif()

  set(GIT_DESCRIBE "v${MAJOR_VERSION}.${MINOR_VERSION}-${VOLK_GIT_COUNT}-${VOLK_GIT_HASH}")
endif()

########################################################################
# Use the logic below to set the version constants
########################################################################
if("${MINOR_VERSION}" STREQUAL "git")
    # VERSION: 1.0git-xxx-gxxxxxxxx
    # DOCVER:  1.0git
    # LIBVER:  1.0git
    set(VERSION "${GIT_DESCRIBE}")
    set(DOCVER  "${MAJOR_VERSION}.0${MINOR_VERSION}")
    set(LIBVER  "${MAJOR_VERSION}.0${MINOR_VERSION}")
    set(RC_MINOR_VERSION "0")
    set(RC_MAINT_VERSION "0")
elseif("${MAINT_VERSION}" STREQUAL "git")
    # VERSION: 1.xgit-xxx-gxxxxxxxx
    # DOCVER:  1.xgit
    # LIBVER:  1.xgit
    set(VERSION "${GIT_DESCRIBE}")
    set(DOCVER  "${MAJOR_VERSION}.${MINOR_VERSION}${MAINT_VERSION}")
    set(LIBVER  "${MAJOR_VERSION}.${MINOR_VERSION}${MAINT_VERSION}")
    math(EXPR RC_MINOR_VERSION "${MINOR_VERSION} - 1")
    set(RC_MAINT_VERSION "0")
else()
    # This is a numbered release.
    # VERSION: 1.1{.x}
    # DOCVER:  1.1{.x}
    # LIBVER:  1.1{.x}
    if("${MAINT_VERSION}" STREQUAL "0")
        set(VERSION "${MAJOR_VERSION}.${MINOR_VERSION}")
    else()
        set(VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
    endif()
    set(DOCVER "${VERSION}")
    set(LIBVER "${VERSION}")
    set(RC_MINOR_VERSION ${MINOR_VERSION})
    set(RC_MAINT_VERSION ${MAINT_VERSION})
endif()
