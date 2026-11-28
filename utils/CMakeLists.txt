# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(front-end-cal)

if(ENABLE_UNIT_TESTING_EXTRA OR ENABLE_SYSTEM_TESTING_EXTRA OR ENABLE_FPGA)
    # obsdiff and rinex2assist require GNSSTk. If it is not installed, or if
    # ENABLE_OWN_GNSSTK is set, they depend on the ExternalProject target
    # defined in tests/CMakeLists.txt, which only exists if tests are enabled.
    if((GNSSTK_FOUND AND NOT ENABLE_OWN_GNSSTK) OR TARGET gnsstk-${GNSSSDR_GNSSTK_LOCAL_VERSION})
        add_subdirectory(rinex-tools)
        add_subdirectory(rinex2assist)
    else()
        message(STATUS "The GNSSTk library is only downloaded and built when tests are enabled.")
        message(STATUS " The obsdiff and rinex2assist utility tools will not be built.")
        message(STATUS " You could build them by setting -DENABLE_UNIT_TESTING=ON or -DENABLE_SYSTEM_TESTING=ON")
    endif()
endif()

if(ENABLE_GNSS_SDR_TUI)
    add_subdirectory(gnss-sdr-tui)
endif()
