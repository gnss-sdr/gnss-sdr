# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(algorithms)
add_subdirectory(core)
add_subdirectory(main)
if(ENABLE_UNIT_TESTING OR ENABLE_SYSTEM_TESTING)
    add_subdirectory(tests)
endif()
add_subdirectory(utils)
