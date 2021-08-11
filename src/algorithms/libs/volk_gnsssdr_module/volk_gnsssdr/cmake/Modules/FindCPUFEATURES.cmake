# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

set(FPHSA_NAME_MISMATCHED ON)

find_library(CPUFEATURES_LIBRARIES
    NAMES cpu_features
    PATHS /usr/lib
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

find_path(CPUFEATURES_INCLUDE_DIR cpu_features_macros.h
    PATHS $ENV{CPUFEATURES_DIR}/include
          $ENV{CPUFEATURES_DIR}
          /usr/include
          /usr/local/include
          ~/Library/Frameworks
          /Library/Frameworks
          /sw/include        # Fink
          /opt/local/include # MacPorts
          /opt/csw/include   # Blastwave
    PATH_SUFFIXES cpu_features
)

if(CPUFEATURES_INCLUDE_DIR AND CPUFEATURES_LIBRARIES)
    if(NOT PACKAGE_VERSION)
        set(PACKAGE_VERSION "")
    endif()
    set(OLD_PACKAGE_VERSION ${PACKAGE_VERSION})
    unset(PACKAGE_VERSION)
    list(GET CPUFEATURES_LIBRARIES 0 FIRST_DIR)
    get_filename_component(CPUFEATURES_LIBRARIES_PATH ${FIRST_DIR} DIRECTORY)
    if(EXISTS ${CPUFEATURES_LIBRARIES_PATH}/cmake/CpuFeatures/CpuFeaturesConfigVersion.cmake)
        include(${CPUFEATURES_LIBRARIES_PATH}/cmake/CpuFeatures/CpuFeaturesConfigVersion.cmake)
    endif()
    if(PACKAGE_VERSION)
        set(CPUFEATURES_VERSION ${PACKAGE_VERSION})
    else()
        set(CPUFEATURES_VERSION "Unknown")
    endif()
    set(PACKAGE_VERSION ${OLD_PACKAGE_VERSION})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CPUFEATURES
    DEFAULT_MSG
    CPUFEATURES_LIBRARIES
    CPUFEATURES_INCLUDE_DIR
)

if(CPUFEATURES_FOUND AND NOT TARGET CpuFeature::cpu_features)
    add_library(CpuFeature::cpu_features STATIC IMPORTED)
    set_target_properties(CpuFeature::cpu_features PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${CPUFEATURES_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${CPUFEATURES_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${CPUFEATURES_LIBRARIES}"
    )
    add_library(cpu_features ALIAS CpuFeature::cpu_features)
endif()

mark_as_advanced(CPUFEATURES_LIBRARIES)
