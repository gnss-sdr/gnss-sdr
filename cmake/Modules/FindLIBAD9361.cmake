# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

#
# Provides the following imported target:
# Iio::ad9361
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

pkg_check_modules(PC_LIBAD9361 libad9361)

if(NOT LIBAD9361_ROOT)
    set(LIBAD9361_ROOT_USER_DEFINED /usr/local)
else()
    set(LIBAD9361_ROOT_USER_DEFINED ${LIBAD9361_ROOT})
endif()
if(DEFINED ENV{LIBAD9361_ROOT})
    set(LIBAD9361_ROOT_USER_DEFINED
        ${LIBAD9361_ROOT_USER_DEFINED}
        $ENV{LIBAD9361_ROOT}
    )
endif()
set(LIBAD9361_ROOT_USER_DEFINED
    ${LIBAD9361_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(LIBAD9361_INCLUDE_DIRS
    NAMES ad9361.h
    HINTS ${PC_LIBAD9361_INCLUDEDIR}
    PATHS ${LIBAD9361_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(LIBAD9361_LIBRARIES
    NAMES ad9361
    HINTS ${PC_LIBAD9361_LIBDIR}
    PATHS ${LIBAD9361_ROOT_USER_DEFINED}/lib
          ${LIBAD9361_ROOT_USER_DEFINED}/lib64
          /usr/lib
          /usr/lib64
          /usr/lib/x86_64-linux-gnu
          /usr/lib/i386-linux-gnu
          /usr/lib/alpha-linux-gnu
          /usr/lib/aarch64-linux-gnu
          /usr/lib/arm-linux-gnueabi
          /usr/lib/arm-linux-gnueabihf
          /usr/lib/hppa-linux-gnu
          /usr/lib/i686-gnu
          /usr/lib/i686-linux-gnu
          /usr/lib/x86_64-kfreebsd-gnu
          /usr/lib/i686-kfreebsd-gnu
          /usr/lib/m68k-linux-gnu
          /usr/lib/mips-linux-gnu
          /usr/lib/mips64el-linux-gnuabi64
          /usr/lib/mipsel-linux-gnu
          /usr/lib/powerpc-linux-gnu
          /usr/lib/powerpc-linux-gnuspe
          /usr/lib/powerpc64-linux-gnu
          /usr/lib/powerpc64le-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/lib/sparc64-linux-gnu
          /usr/lib/x86_64-linux-gnux32
          /usr/lib/sh4-linux-gnu
          /usr/lib/riscv64-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
          /Library/Frameworks/ad9361.framework
)

if(LIBAD9361_LIBRARIES AND APPLE)
    if(LIBAD9361_LIBRARIES MATCHES "framework")
        set(LIBAD9361_LIBRARIES ${LIBAD9361_LIBRARIES}/ad9361)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBAD9361 DEFAULT_MSG LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)

if(PC_LIBAD9361_VERSION)
    set(LIBAD9361_VERSION ${PC_LIBAD9361_VERSION})
endif()

if(LIBAD9361_FOUND AND LIBAD9361_VERSION)
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers (found: v${LIBAD9361_VERSION})"
    )
else()
    set_package_properties(LIBAD9361 PROPERTIES
        DESCRIPTION "A library for interfacing with AD936X RF transceivers"
    )
endif()

set_package_properties(LIBAD9361 PROPERTIES
    URL "https://github.com/analogdevicesinc/libad9361-iio"
)

if(LIBAD9361_FOUND AND NOT TARGET Iio::ad9361)
    add_library(Iio::ad9361 SHARED IMPORTED)
    set_target_properties(Iio::ad9361 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBAD9361_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBAD9361_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBAD9361_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBAD9361_LIBRARIES LIBAD9361_INCLUDE_DIRS)
