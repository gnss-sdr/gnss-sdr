# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(OBS_ADAPTER_SOURCES
    hybrid_observables.cc
)

set(OBS_ADAPTER_HEADERS
    hybrid_observables.h
)

source_group(Headers FILES ${OBS_ADAPTER_HEADERS})

add_library(obs_adapters ${OBS_ADAPTER_SOURCES} ${OBS_ADAPTER_HEADERS})

target_include_directories(obs_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

target_link_libraries(obs_adapters
    PUBLIC
        obs_gr_blocks
        algorithms_libs
        observables_libs
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obs_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET obs_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
