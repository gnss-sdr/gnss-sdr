# Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

add_subdirectory(algorithms)
add_subdirectory(core)
add_subdirectory(main)
if(ENABLE_UNIT_TESTING OR ENABLE_SYSTEM_TESTING)
    add_subdirectory(tests)
endif()
add_subdirectory(utils)
