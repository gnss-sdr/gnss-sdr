# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2021  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause

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

if(GIT_FOUND)
    message(STATUS "Extracting version information from git...")
    # was this info set in the CMake commandline?
    if(NOT GIT_BRANCH)
        # no: try to find it
        execute_process(
            COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_BRANCH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()
    # was this info set in the CMake commandline?
    if(NOT GIT_COMMIT_HASH)
        # Get the latest abbreviated commit hash of the working branch
        execute_process(
            COMMAND ${GIT_EXECUTABLE} log -1 --format=%h
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_COMMIT_HASH
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()
    # Sanitize branch name
    string(REGEX REPLACE "[#!?&|/^$%*]" "_" GIT_BRANCH "${GIT_BRANCH}")
    set(VOLK_GNSSSDR_GIT_BRANCH "${GIT_BRANCH}")
    set(VOLK_GNSSSDR_GIT_HASH "${GIT_COMMIT_HASH}")
else()
    if(GIT_COMMIT_HASH)
        set(VOLK_GNSSSDR_GIT_HASH "${GIT_COMMIT_HASH}")
    else()
        set(VOLK_GNSSSDR_GIT_HASH "unknown")
    endif()

    if(GIT_BRANCH)
        # Sanitize branch name
        string(REGEX REPLACE "[#!?&|/^$%*]" "_" GIT_BRANCH "${GIT_BRANCH}")
        set(VOLK_GNSSSDR_GIT_BRANCH "${GIT_BRANCH}")
    else()
        set(VOLK_GNSSSDR_GIT_BRANCH "unknown")
    endif()
endif()

if("${MAINT_VERSION}" MATCHES "git")
    set(GIT_DESCRIBE "v${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}-${VOLK_GNSSSDR_GIT_BRANCH}-${VOLK_GNSSSDR_GIT_HASH}")
else()
    set(GIT_DESCRIBE "v${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
endif()

########################################################################
# Use the logic below to set the version constants
########################################################################
if("${MINOR_VERSION}" STREQUAL "git")
    # VERSION: 1.0git-xxx-gxxxxxxxx
    # DOCVER:  1.0git
    # LIBVER:  1.0git
    # SOVERSION:  1.0git
    set(VERSION "${GIT_DESCRIBE}")
    set(DOCVER "${MAJOR_VERSION}.0${MINOR_VERSION}")
    set(LIBVER "${MAJOR_VERSION}.0${MINOR_VERSION}")
    set(SOVERSION "${MAJOR_VERSION}.0${MINOR_VERSION}")
elseif("${MAINT_VERSION}" MATCHES "git")
    # VERSION: 1.2.3.git-xxx-gxxxxxxxx
    # DOCVER:  1.2.3.git
    # LIBVER:  1.2.3.git
    # SOVERSION:  1.2.3.git
    set(VERSION "${GIT_DESCRIBE}")
    set(DOCVER "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
    set(LIBVER "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
    set(SOVERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
else()
    # This is a numbered release.
    # VERSION: 1.2{.3}
    # DOCVER:  1.2{.3}
    # SOVERSION:  1.2.3
    if("${MAINT_VERSION}" STREQUAL "0")
        set(VERSION "${MAJOR_VERSION}.${MINOR_VERSION}")
    else()
        set(VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
    endif()
    set(DOCVER "${VERSION}")
    set(SOVERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${MAINT_VERSION}")
endif()
