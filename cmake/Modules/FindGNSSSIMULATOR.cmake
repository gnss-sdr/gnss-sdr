# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2025 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

if(NOT DEFINED GNSSSDR_LIB_PATHS)
    include(GnsssdrFindPaths)
endif()

if(GNSSSIMULATOR_ROOT)
    set(GNSSSIMULATOR_ROOT_USER_DEFINED ${GNSSSIMULATOR_ROOT})
else()
    set(GNSSSIMULATOR_ROOT_USER_DEFINED /usr/local)
endif()
if(DEFINED ENV{GNSSSIMULATOR_ROOT})
    set(GNSSSIMULATOR_ROOT_USER_DEFINED
        ${GNSSSIMULATOR_ROOT_USER_DEFINED}
        $ENV{GNSSSIMULATOR_ROOT}
    )
endif()
if(DEFINED ENV{GNSSSIMULATOR})
    set(GNSSSIMULATOR_ROOT_USER_DEFINED
        ${GNSSSIMULATOR_ROOT_USER_DEFINED}
        $ENV{GNSSSIMULATOR}
    )
endif()
set(GNSSSIMULATOR_ROOT_USER_DEFINED
    ${GNSSSIMULATOR_ROOT_USER_DEFINED}
    ${CMAKE_INSTALL_PREFIX}
)

find_program(SW_GENERATOR_BIN gnss_sim
    PATHS
        ${GNSSSIMULATOR_ROOT_USER_DEFINED}
        /usr
        /usr/local
        ${CMAKE_SYSTEM_PREFIX_PATH}
        ${CMAKE_INSTALL_FULL_BINDIR}
    PATH_SUFFIXES bin
    ONLY_CMAKE_FIND_ROOT_PATH
)

if(SW_GENERATOR_BIN AND CMAKE_CROSSCOMPILING)
    if(CMAKE_SYSROOT)
        string(LENGTH "${CMAKE_SYSROOT}" _gnsssimulator_sysroot_len)
        string(SUBSTRING "${SW_GENERATOR_BIN}" 0 ${_gnsssimulator_sysroot_len} _gnsssimulator_sysroot_prefix)
        if(_gnsssimulator_sysroot_prefix STREQUAL "${CMAKE_SYSROOT}")
            string(SUBSTRING "${SW_GENERATOR_BIN}" ${_gnsssimulator_sysroot_len} -1 SW_GENERATOR_BIN)
        endif()
    elseif(DEFINED ENV{OECORE_TARGET_SYSROOT})
        string(LENGTH "$ENV{OECORE_TARGET_SYSROOT}" _gnsssimulator_sysroot_len)
        string(SUBSTRING "${SW_GENERATOR_BIN}" 0 ${_gnsssimulator_sysroot_len} _gnsssimulator_sysroot_prefix)
        if(_gnsssimulator_sysroot_prefix STREQUAL "$ENV{OECORE_TARGET_SYSROOT}")
            string(SUBSTRING "${SW_GENERATOR_BIN}" ${_gnsssimulator_sysroot_len} -1 SW_GENERATOR_BIN)
        endif()
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNSSSIMULATOR DEFAULT_MSG SW_GENERATOR_BIN)
mark_as_advanced(SW_GENERATOR_BIN)
unset(_gnsssimulator_sysroot_len)
unset(_gnsssimulator_sysroot_prefix)
