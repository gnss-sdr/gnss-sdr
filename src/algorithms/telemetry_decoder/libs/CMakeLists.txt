# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


add_subdirectory(libswiftcnav)

set(TELEMETRY_DECODER_LIB_SOURCES
    viterbi_decoder.cc
)

set(TELEMETRY_DECODER_LIB_HEADERS
    viterbi_decoder.h
    convolutional.h
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
        Gflags::gflags
        Glog::glog
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
