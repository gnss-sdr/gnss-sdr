# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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

if(USE_CMAKE_TARGET_SOURCES)
    add_library(input_filter_gr_blocks STATIC)
    target_sources(input_filter_gr_blocks
        PRIVATE
            ${INPUT_FILTER_GR_BLOCKS_SOURCES}
        PUBLIC
            ${INPUT_FILTER_GR_BLOCKS_HEADERS}
    )
else()
    source_group(Headers FILES ${INPUT_FILTER_GR_BLOCKS_HEADERS})
    add_library(input_filter_gr_blocks
        ${INPUT_FILTER_GR_BLOCKS_SOURCES}
        ${INPUT_FILTER_GR_BLOCKS_HEADERS}
    )
endif()

target_link_libraries(input_filter_gr_blocks
    PUBLIC
        Gnuradio::blocks
        Gnuradio::filter
        Volkgnsssdr::volkgnsssdr
        algorithms_libs
    PRIVATE
        Volk::volk
)

target_include_directories(input_filter_gr_blocks
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(input_filter_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

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
