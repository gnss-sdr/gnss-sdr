# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


########################################################################
# Install python files and apps
########################################################################
include(VolkPython)

volk_python_install(
    FILES
    __init__.py
    cfg.py
    volk_gnsssdr_modtool_generate.py
    DESTINATION ${VOLK_PYTHON_DIR}/volk_gnsssdr_modtool
    COMPONENT "volk_gnsssdr"
)

volk_python_install(
    PROGRAMS
    volk_gnsssdr_modtool
    DESTINATION ${VOLK_RUNTIME_DIR}
    COMPONENT "volk_gnsssdr"
)
