# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2024 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(GNSSSDR_LIB_PATHS)
    return()
endif()

set(GNSSSDR_LIB_PATHS
    /usr/lib
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
    /usr/lib64
    /usr/local/lib
    /usr/local/lib/i386
    /usr/local/lib64
    /opt/local/lib
)
