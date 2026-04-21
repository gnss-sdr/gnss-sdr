# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(TELEMETRY_DECODER_ADAPTER_SOURCES
    telemetry_decoder_adapter.cc
)

set(TELEMETRY_DECODER_ADAPTER_HEADERS
    telemetry_decoder_adapter.h
)

list(SORT TELEMETRY_DECODER_ADAPTER_HEADERS)
list(SORT TELEMETRY_DECODER_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(telemetry_decoder_adapters STATIC)
    target_sources(telemetry_decoder_adapters
        PRIVATE
            ${TELEMETRY_DECODER_ADAPTER_SOURCES}
        PUBLIC
            ${TELEMETRY_DECODER_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${TELEMETRY_DECODER_ADAPTER_HEADERS})
    add_library(telemetry_decoder_adapters
        ${TELEMETRY_DECODER_ADAPTER_SOURCES}
        ${TELEMETRY_DECODER_ADAPTER_HEADERS}
    )
endif()

target_link_libraries(telemetry_decoder_adapters
    PUBLIC
        telemetry_decoder_gr_blocks
        telemetry_decoder_libs
    PRIVATE
        Gnuradio::runtime
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(telemetry_decoder_adapters PRIVATE Gflags::gflags Glog::glog)
    target_compile_definitions(telemetry_decoder_adapters PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(telemetry_decoder_adapters PRIVATE absl::flags absl::log)
endif()

target_include_directories(telemetry_decoder_adapters
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
