# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT COMMAND feature_summary)
    include(FeatureSummary)
endif()

# Locate header
find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h
    HINTS ${SPDLOG_ROOT_DIR}/include
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
)

# Locate library
find_library(SPDLOG_LIBRARY NAMES spdlog spdlogd
    HINTS ${SPDLOG_ROOT_DIR}/lib ${SPDLOG_ROOT_DIR}/lib64
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
find_package_handle_standard_args(SPDLOG DEFAULT_MSG SPDLOG_INCLUDE_DIR SPDLOG_LIBRARY)

set_package_properties(SPDLOG PROPERTIES
    URL "https://github.com/gabime/spdlog"
    DESCRIPTION "Very fast, header-only/compiled, C++ logging library"
)

# Add imported target.
if(SPDLOG_FOUND)
    set(SPDLOG_INCLUDE_DIRS "${SPDLOG_INCLUDE_DIR}")

    if(NOT TARGET spdlog::spdlog)
        add_library(spdlog::spdlog UNKNOWN IMPORTED)
        set_target_properties(spdlog::spdlog PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${SPDLOG_INCLUDE_DIRS}"
        )
        set_property(TARGET spdlog::spdlog APPEND PROPERTY
            IMPORTED_LOCATION "${SPDLOG_LIBRARY}"
        )
    endif()

    if(CMAKE_VERSION VERSION_GREATER 3.11.0)
        target_compile_definitions(spdlog::spdlog INTERFACE -DSPDLOG_FMT_EXTERNAL=1)
    else()
        set_property(TARGET spdlog::spdlog APPEND PROPERTY
            INTERFACE_COMPILE_DEFINITIONS SPDLOG_FMT_EXTERNAL=1
        )
    endif()
endif()
