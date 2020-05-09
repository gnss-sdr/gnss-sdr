# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


set(DATA_TYPE_GR_BLOCKS_SOURCES
    interleaved_byte_to_complex_byte.cc
    interleaved_short_to_complex_short.cc
    interleaved_byte_to_complex_short.cc
)

set(DATA_TYPE_GR_BLOCKS_HEADERS
    interleaved_byte_to_complex_byte.h
    interleaved_short_to_complex_short.h
    interleaved_byte_to_complex_short.h
)

list(SORT DATA_TYPE_GR_BLOCKS_HEADERS)
list(SORT DATA_TYPE_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${DATA_TYPE_GR_BLOCKS_HEADERS})

add_library(data_type_gr_blocks
    ${DATA_TYPE_GR_BLOCKS_SOURCES}
    ${DATA_TYPE_GR_BLOCKS_HEADERS}
)

target_link_libraries(data_type_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Boost::headers
    PRIVATE
        Volk::volk
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(data_type_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

set_property(TARGET data_type_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
