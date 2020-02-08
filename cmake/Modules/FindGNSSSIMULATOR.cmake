# Copyright (C) 2011-2018 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later

find_program(SW_GENERATOR_BIN gnss_sim
    PATHS /usr/bin
          /usr/local/bin
          /opt/local/bin
          ${CMAKE_INSTALL_PREFIX}/bin
          ${GNSSSIMULATOR_ROOT}/bin
          $ENV{GNSSSIMULATOR_ROOT}/bin
    PATH_SUFFIXES bin
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GNSSSIMULATOR DEFAULT_MSG SW_GENERATOR_BIN)
mark_as_advanced(SW_GENERATOR_BIN)
