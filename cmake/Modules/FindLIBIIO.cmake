# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

#
# Provides the following imported target:
# Iio::iio
#

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT PKG_CONFIG_FOUND)
    include(FindPkgConfig)
endif()

pkg_check_modules(PC_LIBIIO libiio)

if(NOT LIBIIO_ROOT)
    set(LIBIIO_ROOT_USER_DEFINED /usr)
else()
    set(LIBIIO_ROOT_USER_DEFINED ${LIBIIO_ROOT})
endif()
if(DEFINED ENV{LIBIIO_ROOT})
    set(LIBIIO_ROOT_USER_DEFINED
        ${LIBIIO_ROOT_USER_DEFINED}
        $ENV{LIBIIO_ROOT}
    )
endif()
set(LIBIIO_ROOT_USER_DEFINED
    ${LIBIIO_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_path(
    LIBIIO_INCLUDE_DIRS
    NAMES iio.h
    HINTS ${PC_LIBIIO_INCLUDEDIR}
    PATHS ${LIBIIO_ROOT_USER_DEFINED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
)

find_library(
    LIBIIO_LIBRARIES
    NAMES iio libiio.so.0
    HINTS ${PC_LIBIIO_LIBDIR}
    PATHS ${LIBIIO_ROOT_USER_DEFINED}/lib
          ${LIBIIO_ROOT_USER_DEFINED}/lib64
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
          /Library/Frameworks/iio.framework/
)

if(LIBIIO_LIBRARIES AND APPLE)
    if(LIBIIO_LIBRARIES MATCHES "framework")
        set(LIBIIO_LIBRARIES ${LIBIIO_LIBRARIES}/iio)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBIIO DEFAULT_MSG LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)

if(PC_LIBIIO_VERSION)
    set(LIBIIO_VERSION ${PC_LIBIIO_VERSION})
endif()

if(LIBIIO_FOUND AND LIBIIO_VERSION)
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices (found: v${LIBIIO_VERSION})"
    )
else()
    set_package_properties(LIBIIO PROPERTIES
        DESCRIPTION "A library for interfacing with Linux IIO devices"
    )
endif()

set_package_properties(LIBIIO PROPERTIES
    URL "https://github.com/analogdevicesinc/libiio"
)

if(LIBIIO_FOUND AND NOT TARGET Iio::iio)
    add_library(Iio::iio SHARED IMPORTED)
    set_target_properties(Iio::iio PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${LIBIIO_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${LIBIIO_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${LIBIIO_LIBRARIES}"
    )
endif()

mark_as_advanced(LIBIIO_LIBRARIES LIBIIO_INCLUDE_DIRS)
