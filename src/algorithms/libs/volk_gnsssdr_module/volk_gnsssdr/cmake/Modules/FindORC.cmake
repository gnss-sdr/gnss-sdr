# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# Copyright (C) 2015-2020  (see AUTHORS file for a list of contributors)
# SPDX-License-Identifier: BSD-3-Clause


set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
set(FPHSA_NAME_MISMATCHED ON)
include(FindPkgConfig)
pkg_check_modules(PC_ORC "orc-0.4 > 0.4.22")

if(NOT ORC_ROOT)
    set(ORC_ROOT_USER_PROVIDED /usr/local)
else()
    set(ORC_ROOT_USER_PROVIDED ${ORC_ROOT})
endif()
if(DEFINED ENV{ORC_ROOT})
    set(ORC_ROOT_USER_PROVIDED
        ${ORC_ROOT_USER_PROVIDED}
        $ENV{ORC_ROOT}
    )
endif()
set(ORC_ROOT_USER_PROVIDED
    ${ORC_ROOT_USER_PROVIDED}
    ${CMAKE_INSTALL_PREFIX}
)
if(PC_ORC_TOOLSDIR)
    set(ORC_ROOT_USER_PROVIDED
        ${ORC_ROOT_USER_PROVIDED}
        ${PC_ORC_TOOLSDIR}
    )
endif()

find_program(ORCC_EXECUTABLE orcc
    HINTS ${ORC_ROOT_USER_PROVIDED}/bin
    PATHS /usr/bin
          /usr/local/bin
          /opt/local/bin
)

find_path(ORC_INCLUDE_DIR
    NAMES orc/orc.h
    HINTS ${PC_ORC_INCLUDEDIR}
    PATHS ${ORC_ROOT_USER_PROVIDED}/include
          /usr/include
          /usr/local/include
          /opt/local/include
    PATH_SUFFIXES orc-0.4
)

find_path(ORC_LIBRARY_DIR
    NAMES ${CMAKE_SHARED_LIBRARY_PREFIX}orc-0.4${CMAKE_SHARED_LIBRARY_SUFFIX}
    HINTS ${PC_ORC_LIBDIR}
    PATHS ${ORC_ROOT_USER_PROVIDED}/lib
          ${ORC_ROOT_USER_PROVIDED}/lib64
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
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
          /usr/lib/hppa-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

find_library(ORC_LIB orc-0.4
    HINTS ${PC_ORC_LIBRARY_DIRS}
    PATHS ${ORC_ROOT_USER_PROVIDED}/lib
          ${ORC_ROOT_USER_PROVIDED}/lib64
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
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
          /usr/lib/hppa-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

find_library(ORC_LIBRARY_STATIC ${CMAKE_STATIC_LIBRARY_PREFIX}orc-0.4${CMAKE_STATIC_LIBRARY_SUFFIX}
    HINTS ${PC_ORC_LIBRARY_DIRS}
    PATHS ${ORC_ROOT}/lib
          ${ORC_ROOT}/lib64
          ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          ${ORC_ROOT_USER_PROVIDED}/lib
          ${ORC_ROOT_USER_PROVIDED}/lib64
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
          /usr/lib/hppa-linux-gnu
          /usr/lib/s390x-linux-gnu
          /usr/local/lib
          /usr/local/lib64
          /opt/local/lib
)

if(PC_ORC_VERSION)
    set(ORC_VERSION ${PC_ORC_VERSION})
endif()

list(APPEND ORC_LIBRARY ${ORC_LIB})

set(ORC_INCLUDE_DIRS ${ORC_INCLUDE_DIR})
set(ORC_LIBRARIES ${ORC_LIBRARY})
set(ORC_LIBRARY_DIRS ${ORC_LIBRARY_DIR})
set(ORC_LIBRARIES_STATIC ${ORC_LIBRARY_STATIC})

include(FindPackageHandleStandardArgs)
if(ENABLE_STATIC_LIBS)
    find_package_handle_standard_args(ORC "orc files" ORC_LIBRARY ORC_INCLUDE_DIR ORCC_EXECUTABLE ORC_LIBRARY_STATIC)
else()
    find_package_handle_standard_args(ORC "orc files" ORC_LIBRARY ORC_INCLUDE_DIR ORCC_EXECUTABLE)
endif()
mark_as_advanced(ORC_INCLUDE_DIR ORC_LIBRARY ORCC_EXECUTABLE ORC_LIBRARY_STATIC)
