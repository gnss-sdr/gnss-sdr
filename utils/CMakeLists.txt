# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(front-end-cal)

if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)
    add_subdirectory(rinex-tools)
    add_subdirectory(rinex2assist)
endif()
