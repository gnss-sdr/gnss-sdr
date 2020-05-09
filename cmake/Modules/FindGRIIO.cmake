# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

#
# Provides the following imported target:
# Gnuradio::iio
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

pkg_check_modules(PC_IIO gnuradio-iio)

if(NOT GRIIO_ROOT)
    set(GRIIO_ROOT_USER_DEFINED /usr)
else()
    set(GRIIO_ROOT_USER_DEFINED ${GRIIO_ROOT})
endif()
if(DEFINED ENV{GRIIO_ROOT})
    set(GRIIO_ROOT_USER_DEFINED
        ${GRIIO_ROOT_USER_DEFINED}
        $ENV{GRIIO_ROOT}
    )
endif()
if(DEFINED ENV{IIO_DIR})
    set(GRIIO_ROOT_USER_DEFINED
        ${GRIIO_ROOT_USER_DEFINED}
        $ENV{IIO_DIR}
    )
endif()
set(GRIIO_ROOT_USER_DEFINED
    ${GRIIO_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)


find_path(IIO_INCLUDE_DIRS
    NAMES gnuradio/iio/api.h
    HINTS ${PC_IIO_INCLUDEDIR}
    PATHS ${GRIIO_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

if(IIO_INCLUDE_DIRS)
    set(GR_IIO_INCLUDE_HAS_GNURADIO TRUE)
else()
    find_path(IIO_INCLUDE_DIRS
        NAMES iio/api.h
        HINTS ${PC_IIO_INCLUDEDIR}
        PATHS ${GRIIO_ROOT_USER_DEFINED}/include
              /usr/include
              /usr/local/include
              /opt/local/include
    )
    set(GR_IIO_INCLUDE_HAS_GNURADIO FALSE)
endif()

find_library(IIO_LIBRARIES
    NAMES gnuradio-iio
    HINTS ${PC_IIO_LIBDIR}
    PATHS ${GRIIO_ROOT_USER_DEFINED}/lib
          ${GRIIO_ROOT_USER_DEFINED}/lib64
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
          /usr/lib/riscv64-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/lib/sparc64-linux-gnu
          /usr/lib/x86_64-linux-gnux32
          /usr/lib/sh4-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GRIIO DEFAULT_MSG IIO_LIBRARIES IIO_INCLUDE_DIRS)

if(PC_IIO_VERSION)
    set(GRIIO_VERSION ${PC_IIO_VERSION})
endif()

set_package_properties(GRIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/gr-iio"
)
if(GRIIO_FOUND AND GRIIO_VERSION)
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio (found: v${GRIIO_VERSION})"
    )
else()
    set_package_properties(GRIIO PROPERTIES
        DESCRIPTION "IIO blocks for GNU Radio"
    )
endif()

if(GRIIO_FOUND AND NOT TARGET Gnuradio::iio)
    add_library(Gnuradio::iio SHARED IMPORTED)
    set_target_properties(Gnuradio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${IIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${IIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${IIO_LIBRARIES}"
    )
endif()

mark_as_advanced(IIO_LIBRARIES IIO_INCLUDE_DIRS GR_IIO_INCLUDE_HAS_GNURADIO)
