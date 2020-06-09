# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
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

set(GCC_MAJOR_SERIES 10 9 8 7 6 5)
set(GCC4_SERIES 4.9.1 4.9 4.8.3 4.8.1 4.7.2 4.7 4.8.2 4.8 4.7 4.6 4.5 4.4.4 4.4)
set(GCC_SERIES ${GCC_MAJOR_SERIES} ${GCC4_SERIES})

find_library(GFORTRAN NAMES gfortran
    PATHS ${GFORTRAN_ROOT_USER_DEFINED}
        /usr/lib64
        /usr/lib/gcc/x86_64-linux-gnu/${GCC_SERIES}  # Debian
        /usr/lib/gcc/i486-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/i586-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/i686-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/arm-linux-gnueabihf/${GCC_SERIES}
        /usr/lib/gcc/aarch64-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/arm-linux-gnueabi/${GCC_SERIES}
        /usr/lib/gcc/alpha-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/riscv64-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/hppa-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/m68k-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/i686-gnu/${GCC_SERIES}
        /usr/lib/gcc/x86_64-kfreebsd-gnu/${GCC_SERIES}
        /usr/lib/gcc/i686-kfreebsd-gnu/${GCC_SERIES}
        /usr/lib/gcc/mips-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/mips64el-linux-gnuabi64/${GCC_SERIES}
        /usr/lib/gcc/mipsel-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/powerpc-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/powerpc-linux-gnuspe/${GCC_SERIES}
        /usr/lib/gcc/powerpc64-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/powerpc64le-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/s390x-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/sparc64-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/x86_64-linux-gnux32/${GCC_SERIES}
        /usr/lib/gcc/sh4-linux-gnu/${GCC_SERIES}
        /usr/lib/gcc/i686-redhat-linux/${GCC_SERIES}  # Fedora
        /usr/lib64/gcc/x86_64-redhat-linux/${GCC_SERIES}
        /usr/lib/gcc/armv7hl-redhat-linux-gnueabi/${GCC_SERIES}
        /usr/lib/gcc/aarch64-redhat-linux/${GCC_SERIES}
        /usr/lib/gcc/ppc64le-redhat-linux/${GCC_SERIES}
        /usr/lib/gcc/s390x-redhat-linux/${GCC_SERIES}
        /usr/lib64/gcc/x86_64-suse-linux/${GCC_SERIES}  # openSUSE
        /usr/lib/gcc/i586-suse-linux/${GCC_SERIES}
        /usr/lib/gcc/x86_64-suse-linux/${GCC_SERIES}
        /usr/lib/gcc/armv6hl-suse-linux-gnueabi/${GCC_SERIES}
        /usr/lib/gcc/armv7hl-suse-linux-gnueabi/${GCC_SERIES}
        /usr/lib64/gcc/aarch64-suse-linux/${GCC_SERIES}
        /usr/lib64/gcc/powerpc64-suse-linux/${GCC_SERIES}
        /usr/lib64/gcc/powerpc64le-suse-linux/${GCC_SERIES}
        /usr/lib64/gcc/riscv64-suse-linux/${GCC_SERIES}
        /usr/lib64/gcc/s390x-suse-linux/${GCC_SERIES}
        /usr/lib/gcc/x86_64-linux-gnu
        /usr/lib/gcc/i686-linux-gnu
        /usr/lib/gcc/i386-linux-gnu
        /usr/lib/gcc/i486-linux-gnu
        /usr/lib/gcc/riscv64-linux-gnu
        /usr/lib/x86_64-linux-gnu
        /usr/lib/i386-linux-gnu
        /usr/lib/arm-linux-gnueabi
        /usr/lib/arm-linux-gnueabihf
        /usr/lib/aarch64-linux-gnu
        /usr/lib/i386-gnu
        /usr/lib/x86_64-kfreebsd-gnu
        /usr/lib/i386-kfreebsd-gnu
        /usr/lib/mips-linux-gnu
        /usr/lib/mips64el-linux-gnuabi64
        /usr/lib/mipsel-linux-gnu
        /usr/lib/powerpc-linux-gnu
        /usr/lib/powerpc64-linux-gnu
        /usr/lib/powerpc64le-linux-gnu
        /usr/lib/s390x-linux-gnu
        /usr/lib/sh4-linux-gnu
        /usr/lib/sparc64-linux-gnu
        /usr/lib/x86_64-linux-gnux32
        /usr/lib/alpha-linux-gnu
        /usr/lib/riscv64-linux-gnu
        /usr/local/lib
        /usr/local/lib64
        /usr/local/lib/i386
)

set_package_properties(GFORTRAN PROPERTIES
    URL "https://gcc.gnu.org/fortran/"
    DESCRIPTION "GNU Fortran library"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GFORTRAN DEFAULT_MSG GFORTRAN)
