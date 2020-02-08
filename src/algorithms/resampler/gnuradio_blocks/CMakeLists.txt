# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


set(RESAMPLER_GR_BLOCKS_SOURCES
    direct_resampler_conditioner_cc.cc
    direct_resampler_conditioner_cs.cc
    direct_resampler_conditioner_cb.cc
)

set(RESAMPLER_GR_BLOCKS_HEADERS
    direct_resampler_conditioner_cc.h
    direct_resampler_conditioner_cs.h
    direct_resampler_conditioner_cb.h
)

list(SORT RESAMPLER_GR_BLOCKS_HEADERS)
list(SORT RESAMPLER_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${RESAMPLER_GR_BLOCKS_HEADERS})

add_library(resampler_gr_blocks
    ${RESAMPLER_GR_BLOCKS_SOURCES}
    ${RESAMPLER_GR_BLOCKS_HEADERS}
)

target_link_libraries(resampler_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Volk::volk
        Boost::headers   # Fix for homebrew
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(resampler_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET resampler_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
