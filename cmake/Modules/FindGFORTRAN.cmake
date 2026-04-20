# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2026 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT CMAKE_INSTALL_LIBDIR)
    include(GNUInstallDirs)
endif()

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

set(GFORTRAN_ROOT_USER_DEFINED "")
if(DEFINED GFORTRAN_ROOT AND NOT "${GFORTRAN_ROOT}" STREQUAL "")
    list(APPEND GFORTRAN_ROOT_USER_DEFINED "${GFORTRAN_ROOT}")
else()
    list(APPEND GFORTRAN_ROOT_USER_DEFINED "/usr/lib")
endif()

if(DEFINED ENV{GFORTRAN_ROOT})
    list(APPEND GFORTRAN_ROOT_USER_DEFINED "$ENV{GFORTRAN_ROOT}")
endif()

set(GFORTRAN_ARCH_PATHS)
list(APPEND GFORTRAN_ARCH_PATHS ${GFORTRAN_ROOT_USER_DEFINED})
if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND GFORTRAN_ARCH_PATHS
        "/usr/lib/gcc/${CMAKE_LIBRARY_ARCHITECTURE}"
        "/usr/lib64/gcc/${CMAKE_LIBRARY_ARCHITECTURE}"
        "/usr/local/lib/gcc/${CMAKE_LIBRARY_ARCHITECTURE}"
    )
endif()

list(APPEND GFORTRAN_ARCH_PATHS
    /usr/lib/gcc/x86_64-linux-gnu
    /usr/lib/gcc/i386-linux-gnu
    /usr/lib/gcc/i486-linux-gnu
    /usr/lib/gcc/i586-linux-gnu
    /usr/lib/gcc/i686-linux-gnu
    /usr/lib/gcc/arm-linux-gnueabihf
    /usr/lib/gcc/aarch64-linux-gnu
    /usr/lib/gcc/arm-linux-gnueabi
    /usr/lib/gcc/alpha-linux-gnu
    /usr/lib/gcc/riscv64-linux-gnu
    /usr/lib/gcc/hppa-linux-gnu
    /usr/lib/gcc/m68k-linux-gnu
    /usr/lib/gcc/i686-gnu
    /usr/lib/gcc/x86_64-kfreebsd-gnu
    /usr/lib/gcc/i686-kfreebsd-gnu
    /usr/lib/gcc/mips-linux-gnu
    /usr/lib/gcc/mips64el-linux-gnuabi64
    /usr/lib/gcc/mipsel-linux-gnu
    /usr/lib/gcc/powerpc-linux-gnu
    /usr/lib/gcc/powerpc-linux-gnuspe
    /usr/lib/gcc/powerpc64-linux-gnu
    /usr/lib/gcc/powerpc64le-linux-gnu
    /usr/lib/gcc/s390x-linux-gnu
    /usr/lib/gcc/sparc64-linux-gnu
    /usr/lib/gcc/x86_64-linux-gnux32
    /usr/lib/gcc/sh4-linux-gnu
    /usr/lib/gcc/i686-redhat-linux
    /usr/lib64/gcc/x86_64-redhat-linux
    /usr/lib/gcc/x86_64-redhat-linux
    /usr/lib/gcc/armv7hl-redhat-linux-gnueabi
    /usr/lib/gcc/aarch64-redhat-linux
    /usr/lib/gcc/ppc64le-redhat-linux
    /usr/lib/gcc/s390x-redhat-linux
    /usr/lib64/gcc/x86_64-suse-linux
    /usr/lib/gcc/i586-suse-linux
    /usr/lib/gcc/x86_64-suse-linux
    /usr/lib/gcc/armv6hl-suse-linux-gnueabi
    /usr/lib/gcc/armv7hl-suse-linux-gnueabi
    /usr/lib/gcc/loongarch64-linux-gnu
    /usr/lib64/gcc/aarch64-suse-linux
    /usr/lib64/gcc/powerpc64-suse-linux
    /usr/lib64/gcc/powerpc64le-suse-linux
    /usr/lib64/gcc/riscv64-suse-linux
    /usr/lib64/gcc/s390x-suse-linux
    ${GNSSSDR_LIB_PATHS}
)

list(REMOVE_DUPLICATES GFORTRAN_ARCH_PATHS)

set(GCC_MAJOR_SERIES)
foreach(_gcc_major RANGE 5 30)
    list(APPEND GCC_MAJOR_SERIES ${_gcc_major})
endforeach()
list(REVERSE GCC_MAJOR_SERIES)
set(GCC4_SERIES 4.9.1 4.9 4.8.3 4.8.2 4.8.1 4.8 4.7.2 4.7 4.6 4.5 4.4.4 4.4)
set(GCC_SERIES ${GCC_MAJOR_SERIES} ${GCC4_SERIES})

find_library(GFORTRAN
    NAMES gfortran
    PATHS
        ${GFORTRAN_ARCH_PATHS}
    PATH_SUFFIXES
        ${GCC_SERIES}
)

set_package_properties(GFORTRAN PROPERTIES
    URL "https://gcc.gnu.org/fortran/"
    DESCRIPTION "GNU Fortran library"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFORTRAN DEFAULT_MSG GFORTRAN)

mark_as_advanced(GFORTRAN)
