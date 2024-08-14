# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

# Tries to find gr-limesdr.
#
# Usage of this module as follows:
#
# find_package(GRLIMESDR)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GrLimeSDR_ROOT_DIR Set this variable to the root installation of
# gr-limesdr if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GRLIMESDR_FOUND System has gr-limesdr libs/headers
# GRLIMESDR_LIBRARIES The gr-limesdr libraries (gnuradio-limesdr)
# GRLIMESDR_INCLUDE_DIR The location of gr-limesdr headers
#
# Provides the following imported target:
# Gnuradio::limesdr
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(GRLIMESDR_PKG QUIET gnuradio-limesdr)

if(NOT GRLIMESDR_ROOT)
    set(GRLIMESDR_ROOT_USER_DEFINED /usr)
else()
    set(GRLIMESDR_ROOT_USER_DEFINED ${GRLIMESDR_ROOT})
endif()
if(DEFINED ENV{GRLIMESDR_ROOT})
    set(GRLIMESDR_ROOT_USER_DEFINED
        ${GRLIMESDR_ROOT_USER_DEFINED}
        $ENV{GRLIMESDR_ROOT}
    )
endif()

find_path(GRLIMESDR_INCLUDE_DIR
    NAMES
        limesdr/source.h
        limesdr/api.h
    HINTS
        ${GRLIMESDR_PKG_INCLUDEDIR}
    PATHS
        ${GRLIMESDR_ROOT_USER_DEFINED}/include
        /usr/include
        /usr/local/include
        /opt/local/include
)

find_library(GRLIMESDR_LIBRARIES
    NAMES
        gnuradio-limesdr
    HINTS
        ${GRLIMESDR_PKG_LIBDIR}
    PATHS
        ${GRLIMESDR_ROOT_USER_DEFINED}/lib
        ${GRLIMESDR_ROOT_USER_DEFINED}/lib64
        ${GNSSSDR_LIB_PATHS}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRLIMESDR DEFAULT_MSG GRLIMESDR_LIBRARIES GRLIMESDR_INCLUDE_DIR)

if(GRLIMESDR_PKG_VERSION)
    set(GRLIMESDR_VERSION_AUX ${GRLIMESDR_PKG_VERSION})
    string(REGEX REPLACE "^v" "" GRLIMESDR_VERSION ${GRLIMESDR_VERSION_AUX})
endif()

set_package_properties(GRLIMESDR PROPERTIES
    URL "https://github.com/myriadrf/gr-limesdr"
)

if(GRLIMESDR_FOUND AND GRLIMESDR_VERSION)
    set_package_properties(GRLIMESDR PROPERTIES
        DESCRIPTION "LimeSDR GNU Radio blocks (found: v${GRLIMESDR_VERSION})"
    )
else()
    set_package_properties(GRLIMESDR PROPERTIES
        DESCRIPTION "LimeSDR GNU Radio blocks"
    )
endif()

if(GRLIMESDR_FOUND AND NOT TARGET Gnuradio::limesdr)
    add_library(Gnuradio::limesdr SHARED IMPORTED)
    set_target_properties(Gnuradio::limesdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GRLIMESDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GRLIMESDR_INCLUDE_DIR};${GRLIMESDR_INCLUDE_DIR}/limesdr"
        INTERFACE_LINK_LIBRARIES "${GRLIMESDR_LIBRARIES}"
    )

    message(STATUS "The (optional) gr-limesdr module has been found.")

    # check for PPS custom version
    file(READ ${GRLIMESDR_INCLUDE_DIR}/limesdr/source.h TMPTXT)
    string(FIND "${TMPTXT}" "enable_PPS_mode" matchres)
    if(${matchres} EQUAL -1)
        message(STATUS " Using standard gr-limesdr library.")
    else()
        set(GRLIMESDR_PPS TRUE)
        message(STATUS " Using custom gr-limesdr library with PPS support.")
    endif()

    # check gr-limesdr branch
    set(_g38_branch TRUE)
    file(STRINGS ${GRLIMESDR_INCLUDE_DIR}/limesdr/source.h _limesdr_header_content)
    foreach(_loop_var IN LISTS _limesdr_header_content)
        string(STRIP "${_loop_var}" _file_line)
        if("static sptr make(std::string serial, int channel_mode, const std::string& filename);" STREQUAL "${_file_line}")
            set(_g38_branch FALSE)
        endif()
        if("make(std::string serial, int channel_mode, const std::string& filename, bool enable_PPS_mode);" STREQUAL "${_file_line}")
            set(_g38_branch FALSE)
        endif()
    endforeach()
    if(${_g38_branch})
        set(GR_LIMESDR_IS_G38_BRANCH TRUE)
    endif()
endif()

mark_as_advanced(GRLIMESDR_LIBRARIES GRLIMESDR_INCLUDE_DIR)
