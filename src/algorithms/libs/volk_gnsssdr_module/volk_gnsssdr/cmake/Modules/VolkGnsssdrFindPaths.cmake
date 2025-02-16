# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(VOLK_GNSSSDR_LIB_PATHS)
    return()
endif()

if(NOT CMAKE_INSTALL_LIBDIR)
    include(GNUInstallDirs)
endif()

set(VOLK_GNSSSDR_LIB_PATHS
    /usr/lib
    /usr/lib64
    /usr/lib/aarch64-linux-gnu
    /usr/lib/alpha-linux-gnu
    /usr/lib/arm-linux-gnueabi
    /usr/lib/arm-linux-gnueabihf
    /usr/lib/hppa-linux-gnu
    /usr/lib/hppa-linux-gnu
    /usr/lib/i386-gnu
    /usr/lib/i386-kfreebsd-gnu
    /usr/lib/i386-linux-gnu
    /usr/lib/loongarch64-linux-gnu
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
    /usr/lib/sh4-linux-gnu
    /usr/lib/sparc64-linux-gnu
    /usr/lib/x86_64-kfreebsd-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/lib/x86_64-linux-gnux32
    /usr/local/lib
    /usr/local/lib64
    /usr/local/lib/i386
    ${CMAKE_INSTALL_FULL_LIBDIR}
    ${CMAKE_SYSTEM_PREFIX_PATH}/${CMAKE_INSTALL_LIBDIR}
    ${CMAKE_INSTALL_PREFIX}/lib
    ${CMAKE_INSTALL_PREFIX}/lib64
)

set(VOLK_GNSSSDR_INCLUDE_PATHS
    /usr/include
    /usr/local/include
    ${CMAKE_INSTALL_FULL_INCLUDEDIR}
    ${CMAKE_SYSTEM_PREFIX_PATH}/include
    ${CMAKE_INSTALL_PREFIX}/include
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(NOT MACOS_PACKAGES_PREFIX)
        include(DetectMacOSPackaging)
    endif()
    set(VOLK_GNSSSDR_LIB_PATHS
        ${VOLK_GNSSSDR_LIB_PATHS}
        ${MACOS_PACKAGES_PREFIX}/${CMAKE_INSTALL_LIBDIR}
        ${MACOS_PACKAGES_PREFIX}/lib
        ${MACOS_PACKAGES_PREFIX}/lib64
    )
    set(VOLK_GNSSSDR_INCLUDE_PATHS ${VOLK_GNSSSDR_INCLUDE_PATHS}
        ${MACOS_PACKAGES_PREFIX}/include
        ~/Library/Frameworks
        /Library/Frameworks
        /sw/include        # Fink
        /opt/csw/include   # Blastwave
    )
endif()

list(REMOVE_DUPLICATES VOLK_GNSSSDR_LIB_PATHS)
list(REMOVE_DUPLICATES VOLK_GNSSSDR_INCLUDE_PATHS)
