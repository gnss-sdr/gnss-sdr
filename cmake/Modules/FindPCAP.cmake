# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2006 Frederic Heem, <frederic.heem@telsey.it>
# SPDX-License-Identifier: BSD-3-Clause

# - Find pcap
# Find the PCAP includes and library
# https://www.tcpdump.org/
#
# The environment variable PCAPDIR allows to specify where to find
# libpcap in non standard location.
#
#  PCAP_INCLUDE_DIRS - where to find pcap.h, etc.
#  PCAP_LIBRARIES   - List of libraries when using pcap.
#  PCAP_FOUND       - True if pcap found.
#
# Provides the following imported target:
# Pcap::pcap
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

pkg_check_modules(PC_PCAP libpcap QUIET)

if(NOT PCAP_ROOT)
    set(PCAP_ROOT_USER_PROVIDED /usr)
else()
    set(PCAP_ROOT_USER_PROVIDED ${PCAP_ROOT})
endif()
if(DEFINED ENV{PCAP_ROOT})
    set(PCAP_ROOT_USER_PROVIDED
        ${PCAP_ROOT_USER_PROVIDED}
        $ENV{PCAP_ROOT}
    )
endif()
if(DEFINED ENV{PCAPDIR})
    set(PCAP_ROOT_USER_PROVIDED
        ${PCAP_ROOT_USER_PROVIDED}
        $ENV{PCAPDIR})
endif()

if(DEFINED ENV{PCAPDIR})
    find_path(PCAP_INCLUDE_DIR
        NAMES
            pcap/pcap.h
            pcap.h
        HINTS
            ${PC_PCAP_INCLUDEDIR}
        PATHS
            ${PCAP_ROOT_USER_PROVIDED}
            ${PCAP_ROOT_USER_PROVIDED}/include
        NO_DEFAULT_PATH
    )
    find_library(PCAP_LIBRARY
        NAMES
            pcap
        HINTS
            ${PC_PCAP_LIBDIR}
        PATHS
            ${PCAP_ROOT_USER_PROVIDED}
            ${PCAP_ROOT_USER_PROVIDED}/lib
        NO_DEFAULT_PATH
    )
else()
    find_path(PCAP_INCLUDE_DIR
        NAMES
            pcap/pcap.h
            pcap.h
        HINTS
            ${PC_PCAP_INCLUDEDIR}
        PATHS
            ${PCAP_ROOT_USER_PROVIDED}/include
            /usr/include
            /usr/local/include
            /opt/local/include
    )
    find_library(PCAP_LIBRARY
        NAMES
            pcap
        HINTS
            ${PC_PCAP_LIBDIR}
        PATHS
            ${PCAP_ROOT_USER_PROVIDED}/lib
            ${GNSSSDR_LIB_PATHS}
    )
endif()

set(PCAP_INCLUDE_DIRS ${PCAP_INCLUDE_DIR})
set(PCAP_LIBRARIES ${PCAP_LIBRARY})

if(PCAP_INCLUDE_DIRS)
    message(STATUS "Pcap include dirs set to ${PCAP_INCLUDE_DIRS}")
else()
    message(STATUS "Pcap include dirs cannot be found.")
endif()

if(PCAP_LIBRARIES)
    message(STATUS "Pcap library set to ${PCAP_LIBRARIES}")
else()
    message(STATUS "Pcap library cannot be found.")
endif()

#Functions
include(CheckFunctionExists)

set(CMAKE_REQUIRED_INCLUDES ${PCAP_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${PCAP_LIBRARIES})
check_function_exists("pcap_breakloop" HAVE_PCAP_BREAKLOOP)
check_function_exists("pcap_datalink_name_to_val" HAVE_PCAP_DATALINK_NAME_TO_VAL)
check_function_exists("pcap_datalink_val_to_name" HAVE_PCAP_DATALINK_VAL_TO_NAME)
check_function_exists("pcap_findalldevs" HAVE_PCAP_FINDALLDEVS)
check_function_exists("pcap_freecode" HAVE_PCAP_FREECODE)
check_function_exists("pcap_get_selectable_fd" HAVE_PCAP_GET_SELECTABLE_FD)
check_function_exists("pcap_lib_version" HAVE_PCAP_LIB_VERSION)
check_function_exists("pcap_list_datalinks" HAVE_PCAP_LIST_DATALINKS)
check_function_exists("pcap_open_dead" HAVE_PCAP_OPEN_DEAD)
check_function_exists("pcap_set_datalink" HAVE_PCAP_SET_DATALINK)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCAP DEFAULT_MSG PCAP_INCLUDE_DIRS PCAP_LIBRARIES)

if(PCAP_FOUND AND PC_PCAP_VERSION)
    set(PCAP_VERSION ${PC_PCAP_VERSION})
endif()

set_package_properties(PCAP PROPERTIES
    URL "https://www.tcpdump.org"
)

if(PCAP_FOUND AND PCAP_VERSION)
    set_package_properties(PCAP PROPERTIES
        DESCRIPTION "A portable C/C++ library for network traffic capture (found: v${PCAP_VERSION})"
    )
else()
    set_package_properties(PCAP PROPERTIES
        DESCRIPTION "A portable C/C++ library for network traffic capture"
    )
endif()

if(PCAP_FOUND AND NOT TARGET Pcap::pcap)
    add_library(Pcap::pcap SHARED IMPORTED)
    set_target_properties(Pcap::pcap PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${PCAP_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${PCAP_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${PCAP_LIBRARIES}"
    )
endif()

mark_as_advanced(PCAP_LIBRARIES PCAP_INCLUDE_DIRS)
