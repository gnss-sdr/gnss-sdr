# Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
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