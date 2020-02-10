# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(OBS_GR_BLOCKS_SOURCES
    hybrid_observables_gs.cc
)

set(OBS_GR_BLOCKS_HEADERS
    hybrid_observables_gs.h
)

source_group(Headers FILES ${OBS_GR_BLOCKS_HEADERS})

add_library(obs_gr_blocks ${OBS_GR_BLOCKS_SOURCES} ${OBS_GR_BLOCKS_HEADERS})

if(${FILESYSTEM_FOUND})
    target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(obs_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(obs_gr_blocks PRIVATE Boost::filesystem)
endif()

target_include_directories(obs_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

target_link_libraries(obs_gr_blocks
    PUBLIC
        Boost::headers
        Gnuradio::blocks
    PRIVATE
        observables_libs
        algorithms_libs
        core_system_parameters
        Gflags::gflags
        Glog::glog
        Matio::matio
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obs_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET obs_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
