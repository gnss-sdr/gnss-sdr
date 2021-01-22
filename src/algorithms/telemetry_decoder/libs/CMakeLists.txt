# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


add_subdirectory(libswiftcnav)

set(TELEMETRY_DECODER_LIB_SOURCES
    tlm_conf.cc
    tlm_utils.cc
    viterbi_decoder.cc
)

set(TELEMETRY_DECODER_LIB_HEADERS
    tlm_conf.h
    viterbi_decoder.h
    convolutional.h
    tlm_utils.h
)

list(SORT TELEMETRY_DECODER_LIB_HEADERS)
list(SORT TELEMETRY_DECODER_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(telemetry_decoder_libs STATIC)
    target_sources(telemetry_decoder_libs
        PRIVATE
            ${TELEMETRY_DECODER_LIB_SOURCES}
        PUBLIC
            ${TELEMETRY_DECODER_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${TELEMETRY_DECODER_LIB_HEADERS})
    add_library(telemetry_decoder_libs
        ${TELEMETRY_DECODER_LIB_SOURCES}
        ${TELEMETRY_DECODER_LIB_HEADERS}
    )
endif()

target_link_libraries(telemetry_decoder_libs
    PUBLIC
        Volkgnsssdr::volkgnsssdr
    PRIVATE
        algorithms_libs
        Gflags::gflags
        Glog::glog
        Matio::matio
)

target_include_directories(telemetry_decoder_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
