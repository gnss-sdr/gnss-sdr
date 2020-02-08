###################################################################
#
#  Copyright (c) 2006 Frederic Heem, <frederic.heem@telsey.it>
#  All rights reserved.
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: BSD-3-Clause
#
###################################################################
# - Find pcap
# Find the PCAP includes and library
# http://www.tcpdump.org/
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

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
include(FindPkgConfig)
pkg_check_modules(PC_PCAP libpcap QUIET)

if(EXISTS $ENV{PCAPDIR})
  find_path(PCAP_INCLUDE_DIR
    NAMES
      pcap/pcap.h
      pcap.h
    HINTS
      ${PC_PCAP_INCLUDEDIR}
    PATHS
      $ENV{PCAPDIR}
      ${PCAP_ROOT}/include
      $ENV{PCAP_ROOT}/include
    NO_DEFAULT_PATH
  )
  find_library(PCAP_LIBRARY
    NAMES
      pcap
    HINTS
      ${PC_PCAP_LIBDIR}
    PATHS
      $ENV{PCAPDIR}
      ${PCAP_ROOT}/lib
      $ENV{PCAP_ROOT}/lib
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
      /usr/include
      /usr/local/include
      /opt/local/include
      ${PCAP_ROOT}/include
      $ENV{PCAP_ROOT}/include
  )
  find_library(PCAP_LIBRARY
    NAMES
      pcap
    HINTS
      ${PC_PCAP_LIBDIR}
    PATHS
      /usr/lib
      /usr/lib64
      /usr/lib/alpha-linux-gnu
      /usr/lib/x86_64-linux-gnu
      /usr/lib/aarch64-linux-gnu
      /usr/lib/arm-linux-gnueabi
      /usr/lib/arm-linux-gnueabihf
      /usr/lib/hppa-linux-gnu
      /usr/lib/i386-linux-gnu
      /usr/lib/m68k-linux-gnu
      /usr/lib/mips-linux-gnu
      /usr/lib/mips64el-linux-gnuabi64
      /usr/lib/mipsel-linux-gnu
      /usr/lib/powerpc-linux-gnuspe
      /usr/lib/powerpc64-linux-gnu
      /usr/lib/powerpc64le-linux-gnu
      /usr/lib/riscv64-linux-gnu
      /usr/lib/s390x-linux-gnu
      /usr/lib/sh4-linux-gnu
      /usr/lib/sparc64-linux-gnu
      /usr/lib/x86_64-linux-gnux32
      /usr/lib/x86_64-kfreebsd-gnu
      /usr/lib/i386-kfreebsd-gnu
      /usr/local/lib
      /usr/local/lib64
      /opt/local/lib
      ${PCAP_ROOT}/lib
      $ENV{PCAP_ROOT}/lib
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

set(OLD_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
set(OLD_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
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
set(CMAKE_REQUIRED_INCLUDES ${OLD_CMAKE_REQUIRED_INCLUDES})
set(CMAKE_REQUIRED_LIBRARIES ${OLD_CMAKE_REQUIRED_LIBRARIES})

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

mark_as_advanced(
  PCAP_LIBRARIES
  PCAP_INCLUDE_DIRS
)
