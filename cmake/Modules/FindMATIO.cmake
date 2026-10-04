# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# FindMATIO
#
# Try to find MATIO library
#
# Once done this will define:
#
#   MATIO_FOUND          - True if MATIO found.
#   MATIO_LIBRARIES      - MATIO libraries.
#   MATIO_INCLUDE_DIRS   - Where to find matio.h, etc.
#   MATIO_VERSION_STRING - Version number as a string (e.g. "1.5.28")
#   MATIO_MAT73_FOUND    - True if MATIO can create MATLAB 7.3 files.
#
# Provides the following imported target:
#   Matio::matio
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

########################################################################
# Build candidate root list
########################################################################
set(_MATIO_ROOT_HINTS)

if(MATIO_ROOT)
    list(APPEND _MATIO_ROOT_HINTS "${MATIO_ROOT}")
endif()

if(DEFINED ENV{MATIO_ROOT} AND NOT "$ENV{MATIO_ROOT}" STREQUAL "")
    list(APPEND _MATIO_ROOT_HINTS "$ENV{MATIO_ROOT}")
endif()

if(CMAKE_INSTALL_PREFIX)
    list(APPEND _MATIO_ROOT_HINTS "${CMAKE_INSTALL_PREFIX}")
endif()

if(VCPKG_INSTALLED_DIR AND VCPKG_TARGET_TRIPLET)
    list(APPEND _MATIO_ROOT_HINTS
        "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}")
endif()

list(REMOVE_DUPLICATES _MATIO_ROOT_HINTS)


########################################################################
# Search paths for headers and libraries
########################################################################
set(_MATIO_INCLUDE_HINTS)
set(_MATIO_LIBRARY_HINTS)

foreach(_matio_prefix ${_MATIO_ROOT_HINTS})
    list(APPEND _MATIO_INCLUDE_HINTS
        "${_matio_prefix}"
        "${_matio_prefix}/include"
    )
    list(APPEND _MATIO_LIBRARY_HINTS
        "${_matio_prefix}/lib"
        "${_matio_prefix}/lib64"
    )
endforeach()

list(APPEND _MATIO_INCLUDE_HINTS ${GNSSSDR_INCLUDE_PATHS})
list(APPEND _MATIO_LIBRARY_HINTS ${GNSSSDR_LIB_PATHS})

list(REMOVE_DUPLICATES _MATIO_INCLUDE_HINTS)
list(REMOVE_DUPLICATES _MATIO_LIBRARY_HINTS)


########################################################################
# Find headers and library
########################################################################
find_path(MATIO_INCLUDE_DIR
    NAMES matio.h
    PATHS ${_MATIO_INCLUDE_HINTS}
    DOC "The MATIO include directory"
)

find_library(MATIO_LIBRARY
    NAMES matio libmatio
    PATHS ${_MATIO_LIBRARY_HINTS}
    DOC "The MATIO library"
)


########################################################################
# Extract version information from MATIO
########################################################################
if(MATIO_INCLUDE_DIR)
    set(MATIO_MAJOR_VERSION 0)
    set(MATIO_MINOR_VERSION 0)
    set(MATIO_RELEASE_LEVEL 0)
    unset(MATIO_CONFIG_FILE)

    if(EXISTS "${MATIO_INCLUDE_DIR}/matio_pubconf.h")
        set(MATIO_CONFIG_FILE "matio_pubconf.h")
    elseif(EXISTS "${MATIO_INCLUDE_DIR}/matioConfig.h")
        set(MATIO_CONFIG_FILE "matioConfig.h")
    endif()

    if(MATIO_CONFIG_FILE)
        file(STRINGS
            "${MATIO_INCLUDE_DIR}/${MATIO_CONFIG_FILE}"
            _matio_header_contents
            REGEX
            "#define MATIO_((MAJOR|MINOR)_VERSION|RELEASE_LEVEL) "
        )

        foreach(line ${_matio_header_contents})
            if(line MATCHES "#define ([A-Z_]+) ([0-9]+)")
                set("${CMAKE_MATCH_1}" "${CMAKE_MATCH_2}")
            endif()
        endforeach()

        unset(_matio_header_contents)
    endif()

    set(MATIO_VERSION_STRING
        "${MATIO_MAJOR_VERSION}.${MATIO_MINOR_VERSION}.${MATIO_RELEASE_LEVEL}"
    )
endif()


########################################################################
# Check MATLAB 7.3 support
########################################################################
if(MATIO_INCLUDE_DIR AND MATIO_LIBRARY)
    if(CMAKE_CROSSCOMPILING)
        # A runtime probe cannot be executed while cross-compiling. Keep the
        # historical behavior in that case and let the toolchain provider
        # ensure that MATIO was built with HDF5/MATLAB 7.3 support.
        set(MATIO_MAT73_FOUND TRUE)
        message(STATUS
            "Matio MATLAB 7.3 support cannot be verified while cross-compiling"
        )
    else()
        include(CheckCSourceRuns)

        set(_MATIO_REQUIRED_INCLUDES_SAVED "${CMAKE_REQUIRED_INCLUDES}")
        set(_MATIO_REQUIRED_LIBRARIES_SAVED "${CMAKE_REQUIRED_LIBRARIES}")
        set(CMAKE_REQUIRED_INCLUDES
            "${MATIO_INCLUDE_DIR};${_MATIO_REQUIRED_INCLUDES_SAVED}"
        )
        set(CMAKE_REQUIRED_LIBRARIES
            "${MATIO_LIBRARY};${_MATIO_REQUIRED_LIBRARIES_SAVED}"
        )

        # MAT_FT_MAT73 is declared even when MATIO is built without HDF5, so
        # compiling against matio.h is not enough. Creating a temporary file
        # is the public API probe that distinguishes a MAT73-capable build.
        unset(MATIO_MAT73_FOUND CACHE)
        unset(MATIO_MAT73_FOUND)
        check_c_source_runs(
            "#include <matio.h>\n#include <stdio.h>\nint main(void)\n{\n    const char filename[] = \"gnsssdr_matio_mat73_probe.mat\";\n    mat_t *matfp = Mat_CreateVer(filename, NULL, MAT_FT_MAT73);\n    int result;\n    if (matfp == NULL)\n        {\n            return 1;\n        }\n    result = Mat_Close(matfp);\n    remove(filename);\n    return result == 0 ? 0 : 1;\n}\n"
            MATIO_MAT73_FOUND
        )

        set(CMAKE_REQUIRED_INCLUDES "${_MATIO_REQUIRED_INCLUDES_SAVED}")
        set(CMAKE_REQUIRED_LIBRARIES "${_MATIO_REQUIRED_LIBRARIES_SAVED}")
        unset(_MATIO_REQUIRED_INCLUDES_SAVED)
        unset(_MATIO_REQUIRED_LIBRARIES_SAVED)
    endif()
endif()


########################################################################
# Standard result handling
########################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MATIO
    REQUIRED_VARS
        MATIO_LIBRARY
        MATIO_INCLUDE_DIR
        MATIO_MAT73_FOUND
    VERSION_VAR
        MATIO_VERSION_STRING
)


########################################################################
# Compatibility variables
########################################################################
if(MATIO_FOUND)
    set(MATIO_LIBRARIES "${MATIO_LIBRARY}")
    set(MATIO_INCLUDE_DIRS "${MATIO_INCLUDE_DIR}")
else()
    set(MATIO_LIBRARIES "")
    set(MATIO_INCLUDE_DIRS "")
endif()

mark_as_advanced(
    MATIO_INCLUDE_DIR
    MATIO_LIBRARY
    MATIO_LIBRARIES
    MATIO_INCLUDE_DIRS
    MATIO_MAT73_FOUND
    MATIO_VERSION_STRING
)


########################################################################
# Package metadata
########################################################################
if(MATIO_FOUND AND MATIO_VERSION_STRING)
    set_package_properties(MATIO PROPERTIES
        DESCRIPTION
        "MATLAB MAT File I/O Library (found: v${MATIO_VERSION_STRING})"
    )
else()
    set_package_properties(MATIO PROPERTIES
        DESCRIPTION "MATLAB MAT File I/O Library"
    )
endif()

set_package_properties(MATIO PROPERTIES
    URL "https://github.com/tbeu/matio"
)


########################################################################
# Imported target
########################################################################
if(MATIO_FOUND AND NOT TARGET Matio::matio)
    add_library(Matio::matio UNKNOWN IMPORTED)
    set_target_properties(Matio::matio PROPERTIES
        IMPORTED_LOCATION "${MATIO_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${MATIO_INCLUDE_DIR}"
    )
endif()


########################################################################
# Cleanup internal variables
########################################################################
unset(_MATIO_ROOT_HINTS)
unset(_MATIO_INCLUDE_HINTS)
unset(_MATIO_LIBRARY_HINTS)
unset(_matio_prefix)
unset(MATIO_CONFIG_FILE)
