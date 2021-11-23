# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(TELEMETRY_DECODER_ADAPTER_SOURCES
    gps_l1_ca_telemetry_decoder.cc
    gps_l2c_telemetry_decoder.cc
    gps_l5_telemetry_decoder.cc
    galileo_e1b_telemetry_decoder.cc
    sbas_l1_telemetry_decoder.cc
    galileo_e5a_telemetry_decoder.cc
    galileo_e5b_telemetry_decoder.cc
    galileo_e6_telemetry_decoder.cc
    glonass_l1_ca_telemetry_decoder.cc
    glonass_l2_ca_telemetry_decoder.cc
    beidou_b1i_telemetry_decoder.cc
    beidou_b3i_telemetry_decoder.cc
)

set(TELEMETRY_DECODER_ADAPTER_HEADERS
    gps_l1_ca_telemetry_decoder.h
    gps_l2c_telemetry_decoder.h
    gps_l5_telemetry_decoder.h
    galileo_e1b_telemetry_decoder.h
    sbas_l1_telemetry_decoder.h
    galileo_e5a_telemetry_decoder.h
    galileo_e5b_telemetry_decoder.h
    galileo_e6_telemetry_decoder.h
    glonass_l1_ca_telemetry_decoder.h
    glonass_l2_ca_telemetry_decoder.h
    beidou_b1i_telemetry_decoder.h
    beidou_b3i_telemetry_decoder.h
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
        Gflags::gflags
        Glog::glog
        Gnuradio::runtime
)

target_include_directories(telemetry_decoder_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
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
