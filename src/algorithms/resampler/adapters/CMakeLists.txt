# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(RESAMPLER_ADAPTER_SOURCES
    direct_resampler_conditioner.cc
    mmse_resampler_conditioner.cc
)

set(RESAMPLER_ADAPTER_HEADERS
    direct_resampler_conditioner.h
    mmse_resampler_conditioner.h
)

list(SORT RESAMPLER_ADAPTER_HEADERS)
list(SORT RESAMPLER_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(resampler_adapters STATIC)
    target_sources(resampler_adapters
        PRIVATE
            ${RESAMPLER_ADAPTER_SOURCES}
        PUBLIC
            ${RESAMPLER_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${RESAMPLER_ADAPTER_HEADERS})
    add_library(resampler_adapters
        ${RESAMPLER_ADAPTER_SOURCES}
        ${RESAMPLER_ADAPTER_HEADERS}
    )
endif()

target_link_libraries(resampler_adapters
    PUBLIC
        resampler_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        Volk::volk
)

target_include_directories(resampler_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(GNURADIO_IS_38_OR_GREATER)
    target_compile_definitions(resampler_adapters PUBLIC -DGR_GREATER_38=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(resampler_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET resampler_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
