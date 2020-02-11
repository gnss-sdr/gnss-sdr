# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


set(INPUT_FILTER_GR_BLOCKS_SOURCES
    beamformer.cc
    pulse_blanking_cc.cc
    notch_cc.cc
    notch_lite_cc.cc
)

set(INPUT_FILTER_GR_BLOCKS_HEADERS
    beamformer.h
    pulse_blanking_cc.h
    notch_cc.h
    notch_lite_cc.h
)

list(SORT INPUT_FILTER_GR_BLOCKS_HEADERS)
list(SORT INPUT_FILTER_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${INPUT_FILTER_GR_BLOCKS_HEADERS})

add_library(input_filter_gr_blocks
    ${INPUT_FILTER_GR_BLOCKS_SOURCES}
    ${INPUT_FILTER_GR_BLOCKS_HEADERS})

target_link_libraries(input_filter_gr_blocks
    PUBLIC
        Gnuradio::blocks
        Gnuradio::filter
    PRIVATE
        Volk::volk
        Log4cpp::log4cpp
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(input_filter_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET input_filter_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
