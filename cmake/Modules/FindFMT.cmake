# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

# Locate header
find_path(FMT_INCLUDE_DIR fmt/core.h
    HINTS ${FMT_ROOT_DIR}/include
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
)

# Locate library
find_library(FMT_LIBRARY NAMES fmt
    HINTS ${FMT_ROOT_DIR}/lib ${FMT_ROOT_DIR}/lib64
    PATHS
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
        /usr/lib/alpha-linux-gnu
        /usr/lib/riscv64-linux-gnu
        /usr/local/lib
        /usr/local/lib64
        /opt/local/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FMT DEFAULT_MSG FMT_INCLUDE_DIR FMT_LIBRARY)

set_package_properties(FMT PROPERTIES
    URL "https://github.com/fmtlib/fmt"
    DESCRIPTION "An open-source formatting library"
)

# Add imported target.
if(FMT_FOUND)
    set(FMT_INCLUDE_DIRS "${FMT_INCLUDE_DIR}")

    if(NOT TARGET fmt::fmt)
        add_library(fmt::fmt UNKNOWN IMPORTED)
        set_target_properties(fmt::fmt PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${FMT_INCLUDE_DIRS}"
        )
        set_property(TARGET fmt::fmt APPEND PROPERTY
            IMPORTED_LOCATION "${FMT_LIBRARY}"
        )
    endif()
endif()
