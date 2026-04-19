# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

set(FPHSA_NAME_MISMATCHED ON)

########################################################################
# Find cpu_features
#
# Provides the following imported target:
#   CpuFeature::cpu_features
########################################################################

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()


########################################################################
# Build candidate root list
########################################################################
set(_CPUFEATURES_ROOT_HINTS)

if(CPUFEATURES_DIR)
    list(APPEND _CPUFEATURES_ROOT_HINTS "${CPUFEATURES_DIR}")
endif()

if(DEFINED ENV{CPUFEATURES_DIR} AND NOT "$ENV{CPUFEATURES_DIR}" STREQUAL "")
    list(APPEND _CPUFEATURES_ROOT_HINTS "$ENV{CPUFEATURES_DIR}")
endif()

if(CMAKE_INSTALL_PREFIX)
    list(APPEND _CPUFEATURES_ROOT_HINTS "${CMAKE_INSTALL_PREFIX}")
endif()
if(DEFINED _CPUFEATURES_ROOT_HINTS)
    list(REMOVE_DUPLICATES _CPUFEATURES_ROOT_HINTS)
endif()


########################################################################
# Search paths for headers and libraries
########################################################################
set(_CPUFEATURES_INCLUDE_HINTS)
set(_CPUFEATURES_LIBRARY_HINTS)

foreach(_cpufeatures_prefix ${_CPUFEATURES_ROOT_HINTS})
    list(APPEND _CPUFEATURES_INCLUDE_HINTS
        "${_cpufeatures_prefix}"
        "${_cpufeatures_prefix}/include"
    )
    list(APPEND _CPUFEATURES_LIBRARY_HINTS
        "${_cpufeatures_prefix}/lib"
        "${_cpufeatures_prefix}/lib64"
    )
endforeach()

list(APPEND _CPUFEATURES_INCLUDE_HINTS ${GNSSSDR_INCLUDE_PATHS})
list(APPEND _CPUFEATURES_LIBRARY_HINTS ${GNSSSDR_LIB_PATHS})

list(REMOVE_DUPLICATES _CPUFEATURES_INCLUDE_HINTS)
list(REMOVE_DUPLICATES _CPUFEATURES_LIBRARY_HINTS)


########################################################################
# Find headers and library
########################################################################
find_library(CPUFEATURES_LIBRARY
    NAMES cpu_features
    PATHS ${_CPUFEATURES_LIBRARY_HINTS}
)

find_path(CPUFEATURES_INCLUDE_DIR
    NAMES cpu_features_macros.h
    PATHS ${_CPUFEATURES_INCLUDE_HINTS}
    PATH_SUFFIXES cpu_features
)


########################################################################
# Standard result handling
########################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CPUFEATURES
    DEFAULT_MSG
    CPUFEATURES_LIBRARY
    CPUFEATURES_INCLUDE_DIR
)


########################################################################
# Determine version
########################################################################
if(CPUFEATURES_FOUND)
    get_filename_component(_CPUFEATURES_LIBRARY_PATH
        "${CPUFEATURES_LIBRARY}" DIRECTORY
    )
    if(EXISTS "${_CPUFEATURES_LIBRARY_PATH}/cmake/CpuFeatures/CpuFeaturesConfigVersion.cmake")
        if(DEFINED PACKAGE_VERSION)
            set(_CPUFEATURES_PACKAGE_VERSION_WAS_DEFINED TRUE)
            set(_CPUFEATURES_OLD_PACKAGE_VERSION "${PACKAGE_VERSION}")
        else()
            set(_CPUFEATURES_PACKAGE_VERSION_WAS_DEFINED FALSE)
        endif()

        unset(PACKAGE_VERSION)
        include(
            "${_CPUFEATURES_LIBRARY_PATH}/cmake/CpuFeatures/CpuFeaturesConfigVersion.cmake"
        )

        if(PACKAGE_VERSION)
            set(CPUFEATURES_VERSION "${PACKAGE_VERSION}")
        else()
            set(CPUFEATURES_VERSION "Unknown")
        endif()

        unset(PACKAGE_VERSION)

        if(_CPUFEATURES_PACKAGE_VERSION_WAS_DEFINED)
            set(PACKAGE_VERSION "${_CPUFEATURES_OLD_PACKAGE_VERSION}")
        else()
            unset(PACKAGE_VERSION)
        endif()

        unset(_CPUFEATURES_PACKAGE_VERSION_WAS_DEFINED)
        unset(_CPUFEATURES_OLD_PACKAGE_VERSION)
    else()
        set(CPUFEATURES_VERSION "Unknown")
    endif()
endif()


########################################################################
# Compatibility variables
########################################################################
set(CPUFEATURES_LIBRARIES "${CPUFEATURES_LIBRARY}")

mark_as_advanced(
    CPUFEATURES_LIBRARY
    CPUFEATURES_LIBRARIES
    CPUFEATURES_INCLUDE_DIR
    CPUFEATURES_VERSION
)


########################################################################
# Define imported target
########################################################################
if(CPUFEATURES_FOUND AND NOT TARGET CpuFeature::cpu_features)
    add_library(CpuFeature::cpu_features UNKNOWN IMPORTED)
    set_target_properties(CpuFeature::cpu_features PROPERTIES
        IMPORTED_LOCATION "${CPUFEATURES_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CPUFEATURES_INCLUDE_DIR}"
    )
endif()


########################################################################
# Cleanup internal variables
########################################################################
unset(_CPUFEATURES_ROOT_HINTS)
unset(_CPUFEATURES_INCLUDE_HINTS)
unset(_CPUFEATURES_LIBRARY_HINTS)
unset(_CPUFEATURES_LIBRARY_PATH)
unset(_cpufeatures_prefix)