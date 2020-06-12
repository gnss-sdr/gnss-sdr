# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

if(USE_CMAKE_TARGET_SOURCES)
    add_library(observables_libs STATIC)
    target_sources(observables_libs
        PRIVATE
            obs_conf.cc
        PUBLIC
            obs_conf.h
    )
else()
    source_group(Headers FILES obs_conf.h)
    add_library(observables_libs obs_conf.cc obs_conf.h)
endif()

target_link_libraries(observables_libs
    PRIVATE
        gnss_sdr_flags
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(observables_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET observables_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
