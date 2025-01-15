# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Find the libunwind library
#
#  LIBUNWIND_FOUND       - True if libunwind was found.
#  LIBUNWIND_LIBRARIES   - The libraries needed to use libunwind
#  LIBUNWIND_INCLUDE_DIR - Location of libunwind.h

# INPUT (Optional):
#  LIBUNWIND_ROOT - path where include + lib of libunwind install is located

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

find_path(LIBUNWIND_INCLUDE_DIR
    NAMES
        libunwind.h
        unwind.h
    HINTS
        /usr
        /usr/local
        /opt/local
    PATH_SUFFIXES include
    PATHS "${LIBUNWIND_ROOT}/include"
)

find_library(LIBUNWIND_GENERIC_LIBRARY
    NAMES
        libunwind
        unwind
    HINTS
        /usr
        /usr/local
        /opt/local
    PATH_SUFFIXES lib lib64
    PATHS
        "${LIBUNWIND_ROOT}/lib"
        "${LIBUNWIND_ROOT}/lib64"
        ${GNSSSDR_LIB_PATHS}
)

if(LIBUNWIND_INCLUDE_DIR)
    if(LIBUNWIND_GENERIC_LIBRARY)
        set(LIBUNWIND_LIBRARIES ${LIBUNWIND_GENERIC_LIBRARY})
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
            set(LIBUNWIND_ARCH "aarch64")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
            set(LIBUNWIND_ARCH "arm")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64|amd64)|(corei7-64)")
            set(LIBUNWIND_ARCH "x86_64")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^i.86$")
            set(LIBUNWIND_ARCH "x86")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^ppc64")
            set(LIBUNWIND_ARCH "ppc64")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^ppc")
            set(LIBUNWIND_ARCH "ppc32")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^mips")
            set(LIBUNWIND_ARCH "mips")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^hppa")
            set(LIBUNWIND_ARCH "hppa")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^ia64")
            set(LIBUNWIND_ARCH "ia64")
        endif()
        if(LIBUNWIND_ARCH)
            find_library(LIBUNWIND_SPECIFIC_LIBRARY
                NAMES
                    libunwind-${LIBUNWIND_ARCH}
                    "unwind-${LIBUNWIND_ARCH}"
                HINTS
                    /usr
                    /usr/local
                    /opt/local
                PATH_SUFFIXES lib lib64
                PATHS "${LIBUNWIND_ROOT}"
            )
            if(NOT LIBUNWIND_SPECIFIC_LIBRARY)
                message(STATUS " -- Failed to find unwind-${LIBUNWIND_ARCH}")
            else()
                set(LIBUNWIND_LIBRARIES ${LIBUNWIND_LIBRARIES} ${LIBUNWIND_SPECIFIC_LIBRARY})
            endif()
        endif()
    endif()
else()
    message(STATUS " -- Could NOT find libunwind.h")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBUNWIND DEFAULT_MSG LIBUNWIND_INCLUDE_DIR)

if(LIBUNWIND_FOUND)
    set(_Unwind_VERSION_HEADER ${LIBUNWIND_INCLUDE_DIR}/libunwind-common.h)
    if(EXISTS ${_Unwind_VERSION_HEADER})
        file(READ ${_Unwind_VERSION_HEADER} _Unwind_VERSION_CONTENTS)
        string(REGEX REPLACE ".*#define UNW_VERSION_MAJOR[ \t]+([0-9]+).*" "\\1"
            LIBUNWIND_VERSION_MAJOR "${_Unwind_VERSION_CONTENTS}")
        string(REGEX REPLACE ".*#define UNW_VERSION_MINOR[ \t]+([0-9]+).*" "\\1"
            LIBUNWIND_VERSION_MINOR "${_Unwind_VERSION_CONTENTS}")
        string(REGEX REPLACE ".*#define UNW_VERSION_EXTRA[ \t]+([0-9]+).*" "\\1"
            LIBUNWIND_VERSION_PATCH "${_Unwind_VERSION_CONTENTS}")
        set(LIBUNWIND_VERSION ${LIBUNWIND_VERSION_MAJOR}.${LIBUNWIND_VERSION_MINOR})
        if(CMAKE_MATCH_0)
            # Third version component may be empty
            set(LIBUNWIND_VERSION ${LIBUNWIND_VERSION}.${LIBUNWIND_VERSION_PATCH})
            set(LIBUNWIND_VERSION_COMPONENTS 3)
        else()
            set(LIBUNWIND_VERSION_COMPONENTS 2)
        endif()
    endif()
endif()

if(LIBUNWIND_FOUND AND NOT TARGET Libunwind::libunwind)
    if(LIBUNWIND_GENERIC_LIBRARY)
        add_library(Libunwind::libunwind SHARED IMPORTED)
        set_target_properties(Libunwind::libunwind PROPERTIES
            IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
            IMPORTED_LOCATION "${LIBUNWIND_GENERIC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBUNWIND_INCLUDE_DIR}"
            INTERFACE_LINK_LIBRARIES "${LIBUNWIND_LIBRARIES}"
        )
    else()
        add_library(Libunwind::libunwind INTERFACE IMPORTED)
        set_target_properties(Libunwind::libunwind PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${LIBUNWIND_INCLUDE_DIR}"
        )
    endif()
endif()

set_package_properties(LIBUNWIND PROPERTIES
    URL "https://www.nongnu.org/libunwind/"
)
if(LIBUNWIND_VERSION)
    set_package_properties(LIBUNWIND PROPERTIES
        DESCRIPTION "Portable and efficient C programming interface to determine the call-chain of a program (found: v${LIBUNWIND_VERSION})"
    )
else()
    set_package_properties(LIBUNWIND PROPERTIES
        DESCRIPTION "Portable and efficient C programming interface to determine the call-chain of a program"
    )
endif()

mark_as_advanced(LIBUNWIND_LIBRARIES LIBUNWIND_INCLUDE_DIR)
