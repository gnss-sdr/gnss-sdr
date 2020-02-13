# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Install python files and apps
########################################################################
include(VolkPython)

VOLK_PYTHON_INSTALL(
    FILES
    __init__.py
    cfg.py
    volk_gnsssdr_modtool_generate.py
    DESTINATION ${VOLK_PYTHON_DIR}/volk_gnsssdr_modtool
    COMPONENT "volk_gnsssdr"
)

VOLK_PYTHON_INSTALL(
    PROGRAMS
    volk_gnsssdr_modtool
    DESTINATION ${VOLK_RUNTIME_DIR}
    COMPONENT "volk_gnsssdr"
)
