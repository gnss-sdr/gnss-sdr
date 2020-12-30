# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(INPUT_FILTER_ADAPTER_SOURCES
    fir_filter.cc
    freq_xlating_fir_filter.cc
    beamformer_filter.cc
    pulse_blanking_filter.cc
    notch_filter.cc
    notch_filter_lite.cc
)

set(INPUT_FILTER_ADAPTER_HEADERS
    fir_filter.h
    freq_xlating_fir_filter.h
    beamformer_filter.h
    pulse_blanking_filter.h
    notch_filter.h
    notch_filter_lite.h
)

list(SORT INPUT_FILTER_ADAPTER_HEADERS)
list(SORT INPUT_FILTER_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(input_filter_adapters STATIC)
    target_sources(input_filter_adapters
        PRIVATE
            ${INPUT_FILTER_ADAPTER_SOURCES}
        PUBLIC
            ${INPUT_FILTER_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${INPUT_FILTER_ADAPTER_HEADERS})
    add_library(input_filter_adapters
        ${INPUT_FILTER_ADAPTER_SOURCES}
        ${INPUT_FILTER_ADAPTER_HEADERS}
    )
endif()

target_link_libraries(input_filter_adapters
    PUBLIC
        algorithms_libs
        input_filter_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        Volk::volk
)

if(GNURADIO_IS_38_OR_GREATER)
    target_compile_definitions(input_filter_adapters PUBLIC -DGR_GREATER_38=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(input_filter_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET input_filter_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
