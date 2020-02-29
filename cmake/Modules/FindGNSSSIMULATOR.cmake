# Copyright (C) 2011-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

if(GNSSSIMULATOR_ROOT)
    set(GNSSSIMULATOR_ROOT_USER_DEFINED ${GNSSSIMULATOR_ROOT})
else()
    set(GNSSSIMULATOR_ROOT_USER_DEFINED /usr/local/bin)
endif()
if(DEFINED ENV{GNSSSIMULATOR_ROOT})
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
        /opt/local
    PATH_SUFFIXES bin
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNSSSIMULATOR DEFAULT_MSG SW_GENERATOR_BIN)
mark_as_advanced(SW_GENERATOR_BIN)
