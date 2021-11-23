# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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

if(USE_CMAKE_TARGET_SOURCES)
    add_library(data_type_gr_blocks STATIC)
    target_sources(data_type_gr_blocks
        PRIVATE
            ${DATA_TYPE_GR_BLOCKS_SOURCES}
        PUBLIC
            ${DATA_TYPE_GR_BLOCKS_HEADERS}
    )
else()
    source_group(Headers FILES ${DATA_TYPE_GR_BLOCKS_HEADERS})
    add_library(data_type_gr_blocks
        ${DATA_TYPE_GR_BLOCKS_SOURCES}
        ${DATA_TYPE_GR_BLOCKS_HEADERS}
    )
endif()

target_link_libraries(data_type_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Boost::headers
    PRIVATE
        Volk::volk
)

target_include_directories(data_type_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(data_type_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(GNURADIO_USES_SPDLOG)
    target_link_libraries(data_type_gr_blocks
        PUBLIC
            fmt::fmt
            spdlog::spdlog
    )
endif()

set_property(TARGET data_type_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
