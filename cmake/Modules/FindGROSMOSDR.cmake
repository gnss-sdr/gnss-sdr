# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Tries to find gr-osmosdr.
#
# Usage of this module as follows:
#
# find_package(GROSMOSDR)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
# GrOsmoSDR_ROOT_DIR Set this variable to the root installation of
# gr-osmosdr if the module has problems finding
# the proper installation path.
#
# Variables defined by this module:
#
# GROSMOSDR_FOUND System has gr-osmosdr libs/headers
# GROSMOSDR_LIBRARIES The gr-osmosdr libraries (gnuradio-osmosdr)
# GROSMOSDR_INCLUDE_DIR The location of gr-osmosdr headers
#
# Provides the following imported target:
# Gnuradio::osmosdr
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

pkg_check_modules(GROSMOSDR_PKG gnuradio-osmosdr)

if(NOT GROSMOSDR_ROOT)
    set(GROSMOSDR_ROOT_USER_DEFINED /usr)
else()
    set(GROSMOSDR_ROOT_USER_DEFINED ${GROSMOSDR_ROOT})
endif()
if(DEFINED ENV{GROSMOSDR_ROOT})
    set(GROSMOSDR_ROOT_USER_DEFINED
        ${GROSMOSDR_ROOT_USER_DEFINED}
        $ENV{GROSMOSDR_ROOT}
    )
endif()

find_path(GROSMOSDR_INCLUDE_DIR
    NAMES
        osmosdr/source.h
        osmosdr/api.h
    HINTS
        ${GROSMOSDR_PKG_INCLUDEDIR}
    PATHS
        ${GROSMOSDR_ROOT_USER_DEFINED}/include
        /usr/include
        /usr/local/include
        /opt/local/include
)

find_library(GROSMOSDR_LIBRARIES
    NAMES
        gnuradio-osmosdr
    HINTS
        ${GROSMOSDR_PKG_LIBDIR}
    PATHS
        ${GROSMOSDR_ROOT_USER_DEFINED}/lib
        ${GROSMOSDR_ROOT_USER_DEFINED}/lib64
        /usr/lib
        /usr/lib64
        /usr/lib/x86_64-linux-gnu
        /usr/lib/i386-linux-gnu
        /usr/lib/arm-linux-gnueabihf
        /usr/lib/arm-linux-gnueabi
        /usr/lib/aarch64-linux-gnu
        /usr/lib/mipsel-linux-gnu
        /usr/lib/mips-linux-gnu
        /usr/lib/mips64el-linux-gnuabi64
        /usr/lib/powerpc-linux-gnu
        /usr/lib/powerpc64-linux-gnu
        /usr/lib/powerpc64le-linux-gnu
        /usr/lib/powerpc-linux-gnuspe
        /usr/lib/hppa-linux-gnu
        /usr/lib/s390x-linux-gnu
        /usr/lib/i386-gnu
        /usr/lib/hppa-linux-gnu
        /usr/lib/x86_64-kfreebsd-gnu
        /usr/lib/i386-kfreebsd-gnu
        /usr/lib/m68k-linux-gnu
        /usr/lib/sh4-linux-gnu
        /usr/lib/sparc64-linux-gnu
        /usr/lib/x86_64-linux-gnux32
        /usr/lib/riscv64-linux-gnu
        /usr/lib/alpha-linux-gnu
        /usr/local/lib
        /usr/local/lib64
        /opt/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GROSMOSDR DEFAULT_MSG GROSMOSDR_LIBRARIES GROSMOSDR_INCLUDE_DIR)

if(GROSMOSDR_PKG_VERSION)
    set(GROSMOSDR_VERSION_AUX ${GROSMOSDR_PKG_VERSION})
    string(REGEX REPLACE "^v" "" GROSMOSDR_VERSION ${GROSMOSDR_VERSION_AUX})
endif()

set_package_properties(GROSMOSDR PROPERTIES
    URL "https://osmocom.org/projects/gr-osmosdr/wiki"
)

if(GROSMOSDR_FOUND AND GROSMOSDR_VERSION)
    set_package_properties(GROSMOSDR PROPERTIES
        DESCRIPTION "osmocom GNU Radio blocks (found: v${GROSMOSDR_VERSION})"
    )
else()
    set_package_properties(GROSMOSDR PROPERTIES
        DESCRIPTION "osmocom GNU Radio blocks"
    )
endif()

if(GROSMOSDR_FOUND AND NOT TARGET Gnuradio::osmosdr)
    add_library(Gnuradio::osmosdr SHARED IMPORTED)
    set_target_properties(Gnuradio::osmosdr PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GROSMOSDR_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GROSMOSDR_INCLUDE_DIR};${GROSMOSDR_INCLUDE_DIR}/osmosdr"
        INTERFACE_LINK_LIBRARIES "${GROSMOSDR_LIBRARIES}"
    )
endif()

mark_as_advanced(GROSMOSDR_LIBRARIES GROSMOSDR_INCLUDE_DIR)
