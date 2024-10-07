# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# - Try to find GFlags
#
# The following CMake and environment variables are optionally searched
# for defaults:
# GFLAGS_ROOT: Base directory where all GFlags components are found
#
# The following are set after configuration is done:
# GFlags_FOUND
# GFlags_INCLUDE_DIRS
# GFlags_LIBS
# GFlags_LIBRARY_DIRS
#
# Provides the following imported target:
# Gflags::gflags
#


if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT GFLAGS_ROOT)
    set(GFLAGS_ROOT_USER_PROVIDED /usr/local)
else()
    set(GFLAGS_ROOT_USER_PROVIDED ${GFLAGS_ROOT})
endif()
if(DEFINED ENV{GFLAGS_ROOT})
    set(GFLAGS_ROOT_USER_PROVIDED
        ${GFLAGS_ROOT_USER_PROVIDED}
        $ENV{GFLAGS_ROOT}
    )
endif()

if(APPLE)
    find_path(GFlags_ROOT_DIR
        libgflags.dylib
        PATHS
            ${GFLAGS_ROOT_USER_PROVIDED}/lib
            /usr/local/lib
            /opt/local/lib
            /opt/homebrew/opt/gflags/lib
    )
else()
    find_path(GFlags_ROOT_DIR
        libgflags.so
        PATHS
            ${GFLAGS_ROOT_USER_PROVIDED}/lib
            ${GFLAGS_ROOT_USER_PROVIDED}/lib64
            ${GNSSSDR_LIB_PATHS}
    )
endif()

if(GFlags_ROOT_DIR)
    unset(GFlags_INCLUDE_DIRS CACHE)
    # We are testing only a couple of files in the include directories
    find_path(GFlags_INCLUDE_DIRS
        gflags/gflags.h
        PATHS
            ${GFlags_ROOT_DIR}/src
            ${GFLAGS_ROOT_USER_PROVIDED}/include
            /usr/include
            /usr/local/include
            /opt/local/include
            /opt/homebrew/opt/gflags/include
    )

    # Find the libraries
    set(GFlags_LIBRARY_DIRS ${GFlags_ROOT_DIR})

    find_library(GFlags_lib gflags ${GFlags_LIBRARY_DIRS})
    if(EXISTS ${GFlags_INCLUDE_DIRS}/gflags/gflags_gflags.h)
        set(GFLAGS_GREATER_20 TRUE)
    else()
        set(GFLAGS_GREATER_20 FALSE)
    endif()
    message(STATUS "gflags library found at ${GFlags_lib}")
    set(GFlags_LIBS ${GFlags_lib})
    set(GFlags_FOUND true)
    mark_as_advanced(GFlags_INCLUDE_DIRS)
else()
    message(STATUS "Cannot find gflags")
    set(GFlags_FOUND false)
endif()

if(GFlags_LIBS AND GFlags_INCLUDE_DIRS)
    if(NOT PACKAGE_VERSION)
        set(PACKAGE_VERSION "")
    endif()
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET GFlags_LIBS 0 FIRST_DIR)
    get_filename_component(GFlags_LIBS_DIR ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${GFlags_LIBS_DIR}/cmake/gflags/gflags-config-version.cmake)
        include(${GFlags_LIBS_DIR}/cmake/gflags/gflags-config-version.cmake)
    endif()
    if(NOT PACKAGE_VERSION)
        if(EXISTS ${GFlags_INCLUDE_DIRS}/google/gflags.h)
            set(PACKAGE_VERSION "2.0")
        endif()
    endif()
    if(PACKAGE_VERSION)
        set(GFLAGS_VERSION ${PACKAGE_VERSION})
        set_package_properties(GFLAGS PROPERTIES
            DESCRIPTION "C++ library that implements commandline flags processing (found: v${GFLAGS_VERSION})"
        )
    else()
        set_package_properties(GFLAGS PROPERTIES
            DESCRIPTION "C++ library that implements commandline flags processing"
        )
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
else()
    set_package_properties(GFLAGS PROPERTIES
        DESCRIPTION "C++ library that implements commandline flags processing"
    )
endif()

set_package_properties(GFLAGS PROPERTIES
    URL "https://github.com/gflags/gflags"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFLAGS DEFAULT_MSG GFlags_LIBS GFlags_INCLUDE_DIRS)

if(GFLAGS_VERSION)
    if(${GFLAGS_VERSION} VERSION_LESS "${GNSSSDR_GFLAGS_MIN_VERSION}")
        set(GFLAGS_FOUND FALSE)
        unset(GFlags_LIBS CACHE)
        unset(GFlags_INCLUDE_DIRS CACHE)
    endif()
endif()

if(GFLAGS_FOUND AND NOT TARGET Gflags::gflags)
    add_library(Gflags::gflags SHARED IMPORTED)
    set_target_properties(Gflags::gflags PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GFlags_LIBS}"
        INTERFACE_INCLUDE_DIRECTORIES "${GFlags_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${GFlags_LIBS}"
    )
endif()
