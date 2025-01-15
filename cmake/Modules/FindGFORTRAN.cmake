# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

if(NOT GNSSSDR_LIB_PATHS)
    include(GnsssdrLibPaths)
endif()

if(NOT GFORTRAN_ROOT)
    set(GFORTRAN_ROOT_USER_DEFINED /usr/lib)
else()
    set(GFORTRAN_ROOT_USER_DEFINED ${GFORTRAN_ROOT})
endif()
if(DEFINED ENV{GFORTRAN_ROOT})
    set(GFORTRAN_ROOT_USER_DEFINED
        $ENV{GFORTRAN_ROOT}
        ${GFORTRAN_ROOT_USER_DEFINED}
    )
endif()

set(GCC_MAJOR_SERIES 15 14 13 12 11 10 9 8 7 6 5)
set(GCC4_SERIES 4.9.1 4.9 4.8.3 4.8.1 4.7.2 4.7 4.8.2 4.8 4.7 4.6 4.5 4.4.4 4.4)
set(GCC_SERIES ${GCC_MAJOR_SERIES} ${GCC4_SERIES})

find_library(GFORTRAN NAMES gfortran
    PATHS ${GFORTRAN_ROOT_USER_DEFINED}
        /usr/lib/gcc/x86_64-linux-gnu  # Debian
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
        /usr/lib/gcc/i686-redhat-linux  # Fedora
        /usr/lib64/gcc/x86_64-redhat-linux
        /usr/lib/gcc/x86_64-redhat-linux
        /usr/lib/gcc/armv7hl-redhat-linux-gnueabi
        /usr/lib/gcc/aarch64-redhat-linux
        /usr/lib/gcc/ppc64le-redhat-linux
        /usr/lib/gcc/s390x-redhat-linux
        /usr/lib64/gcc/x86_64-suse-linux  # openSUSE
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
    PATH_SUFFIXES
        ${GCC_SERIES}
)

set_package_properties(GFORTRAN PROPERTIES
    URL "https://gcc.gnu.org/fortran/"
    DESCRIPTION "GNU Fortran library"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFORTRAN DEFAULT_MSG GFORTRAN)
