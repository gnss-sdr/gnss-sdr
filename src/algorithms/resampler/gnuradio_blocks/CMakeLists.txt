# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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

if(USE_CMAKE_TARGET_SOURCES)
    add_library(resampler_gr_blocks STATIC)
    target_sources(resampler_gr_blocks
        PRIVATE
            ${RESAMPLER_GR_BLOCKS_SOURCES}
        PUBLIC
            ${RESAMPLER_GR_BLOCKS_HEADERS}
    )
else()
    source_group(Headers FILES ${RESAMPLER_GR_BLOCKS_HEADERS})
    add_library(resampler_gr_blocks
        ${RESAMPLER_GR_BLOCKS_SOURCES}
        ${RESAMPLER_GR_BLOCKS_HEADERS}
    )
endif()

target_link_libraries(resampler_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Boost::headers   # Fix for homebrew
    PRIVATE
        Volk::volk
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(resampler_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(GNURADIO_IS_38_OR_GREATER)
    target_compile_definitions(resampler_gr_blocks
        PUBLIC -DGR_GREATER_38=1)
endif()

target_include_directories(resampler_gr_blocks
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
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
