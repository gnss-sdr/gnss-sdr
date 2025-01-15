# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Tries to find Gperftools.
#
# Usage of this module as follows:
#
# find_package(GPERFTOOLS)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GPERFTOOLS_ROOT Set this variable to the root installation of
# Gperftools if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GPERFTOOLS_FOUND System has Gperftools libs/headers
# GPERFTOOLS_LIBRARIES The Gperftools libraries (tcmalloc & profiler)
# GPERFTOOLS_INCLUDE_DIR The location of Gperftools headers
#
# Provides the following imported targets:
# Gperftools::tcmalloc
# Gperftools::profiler
# Gperftools::gperftools
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GPERFTOOLS_ROOT)
    set(GPERFTOOLS_ROOT_USER_DEFINED /usr/local)
else()
    set(GPERFTOOLS_ROOT_USER_DEFINED ${GPERFTOOLS_ROOT})
endif()
if(DEFINED ENV{GPERFTOOLS_ROOT})
    set(GPERFTOOLS_ROOT_USER_DEFINED
        ${GPERFTOOLS_ROOT_USER_DEFINED}
        $ENV{GPERFTOOLS_ROOT}
    )
endif()
if(Gperftools_ROOT_DIR)
    set(GPERFTOOLS_ROOT_USER_DEFINED
        ${GPERFTOOLS_ROOT_USER_DEFINED}
        ${Gperftools_ROOT_DIR}
    )
endif()


find_library(GPERFTOOLS_TCMALLOC
    NAMES tcmalloc
    PATHS ${GPERFTOOLS_ROOT_USER_DEFINED}/lib
          ${GPERFTOOLS_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

find_library(GPERFTOOLS_PROFILER
    NAMES profiler
    PATHS ${GPERFTOOLS_ROOT_USER_DEFINED}/lib
          ${GPERFTOOLS_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

find_library(GPERFTOOLS_TCMALLOC_AND_PROFILER
    NAMES tcmalloc_and_profiler
    PATHS ${GPERFTOOLS_ROOT_USER_DEFINED}/lib
          ${GPERFTOOLS_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

find_path(GPERFTOOLS_INCLUDE_DIR
    NAMES gperftools/heap-profiler.h
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
          ${GPERFTOOLS_ROOT_USER_DEFINED}/include
)

set(GPERFTOOLS_LIBRARIES ${GPERFTOOLS_TCMALLOC_AND_PROFILER})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    GPERFTOOLS
    DEFAULT_MSG
    GPERFTOOLS_LIBRARIES
    GPERFTOOLS_INCLUDE_DIR
    GPERFTOOLS_TCMALLOC
    GPERFTOOLS_PROFILER
)

if(GPERFTOOLS_FOUND AND NOT TARGET Gperftools::tcmalloc)
    add_library(Gperftools::tcmalloc SHARED IMPORTED)
    set_target_properties(Gperftools::tcmalloc PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPERFTOOLS_TCMALLOC}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPERFTOOLS_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${GPERFTOOLS_TCMALLOC}"
    )
endif()

if(GPERFTOOLS_FOUND AND NOT TARGET Gperftools::profiler)
    add_library(Gperftools::profiler SHARED IMPORTED)
    set_target_properties(Gperftools::profiler PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPERFTOOLS_PROFILER}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPERFTOOLS_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${GPERFTOOLS_PROFILER}"
    )
endif()

if(GPERFTOOLS_FOUND AND NOT TARGET Gperftools::gperftools)
    add_library(Gperftools::gperftools SHARED IMPORTED)
    set_target_properties(Gperftools::gperftools PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GPERFTOOLS_TCMALLOC_AND_PROFILER}"
        INTERFACE_INCLUDE_DIRECTORIES "${GPERFTOOLS_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${GPERFTOOLS_TCMALLOC_AND_PROFILER}"
    )
endif()

set_package_properties(GPERFTOOLS PROPERTIES
    URL "https://github.com/gperftools/gperftools"
    DESCRIPTION "Collection of performance analysis tools"
)

mark_as_advanced(
    GPERFTOOLS_TCMALLOC
    GPERFTOOLS_PROFILER
    GPERFTOOLS_TCMALLOC_AND_PROFILER
    GPERFTOOLS_LIBRARIES
    GPERFTOOLS_INCLUDE_DIR
)
