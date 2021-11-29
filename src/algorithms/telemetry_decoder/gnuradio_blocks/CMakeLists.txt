# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(TELEMETRY_DECODER_GR_BLOCKS_SOURCES
    gps_l1_ca_telemetry_decoder_gs.cc
    gps_l2c_telemetry_decoder_gs.cc
    gps_l5_telemetry_decoder_gs.cc
    sbas_l1_telemetry_decoder_gs.cc
    glonass_l1_ca_telemetry_decoder_gs.cc
    glonass_l2_ca_telemetry_decoder_gs.cc
    galileo_telemetry_decoder_gs.cc
    beidou_b1i_telemetry_decoder_gs.cc
    beidou_b3i_telemetry_decoder_gs.cc
)

set(TELEMETRY_DECODER_GR_BLOCKS_HEADERS
    gps_l1_ca_telemetry_decoder_gs.h
    gps_l2c_telemetry_decoder_gs.h
    gps_l5_telemetry_decoder_gs.h
    sbas_l1_telemetry_decoder_gs.h
    glonass_l1_ca_telemetry_decoder_gs.h
    glonass_l2_ca_telemetry_decoder_gs.h
    galileo_telemetry_decoder_gs.h
    beidou_b1i_telemetry_decoder_gs.h
    beidou_b3i_telemetry_decoder_gs.h
)

list(SORT TELEMETRY_DECODER_GR_BLOCKS_HEADERS)
list(SORT TELEMETRY_DECODER_GR_BLOCKS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(telemetry_decoder_gr_blocks STATIC)
    target_sources(telemetry_decoder_gr_blocks
        PRIVATE
            ${TELEMETRY_DECODER_GR_BLOCKS_SOURCES}
        PUBLIC
            ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS}
    )
else()
    source_group(Headers FILES ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS})
    add_library(telemetry_decoder_gr_blocks
        ${TELEMETRY_DECODER_GR_BLOCKS_SOURCES}
        ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS}
    )
endif()

target_link_libraries(telemetry_decoder_gr_blocks
    PUBLIC
        telemetry_decoder_libswiftcnav
        telemetry_decoder_libs
        algorithms_libs
        core_libs
        core_system_parameters
        Gnuradio::runtime
        Boost::headers
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(telemetry_decoder_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(GNURADIO_USES_SPDLOG)
    target_link_libraries(telemetry_decoder_gr_blocks
        PUBLIC
            fmt::fmt
            spdlog::spdlog
    )
endif()

target_include_directories(telemetry_decoder_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

target_include_directories(telemetry_decoder_gr_blocks
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs  # for gnss_sdr_make_unique.h
)

if(has_rotl)
    target_compile_definitions(telemetry_decoder_gr_blocks
        PRIVATE -DCOMPILER_HAS_ROTL=1
    )
endif()

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(telemetry_decoder_gr_blocks
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(telemetry_decoder_gr_blocks
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
