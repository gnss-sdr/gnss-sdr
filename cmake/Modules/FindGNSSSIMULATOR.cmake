# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2011-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

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
