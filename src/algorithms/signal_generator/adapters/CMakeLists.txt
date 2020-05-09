# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(SIGNAL_GENERATOR_ADAPTER_SOURCES signal_generator.cc)
set(SIGNAL_GENERATOR_ADAPTER_HEADERS signal_generator.h)

source_group(Headers FILES ${SIGNAL_GENERATOR_ADAPTER_HEADERS})

add_library(signal_generator_adapters
    ${SIGNAL_GENERATOR_ADAPTER_SOURCES}
    ${SIGNAL_GENERATOR_ADAPTER_HEADERS}
)

target_link_libraries(signal_generator_adapters
    PUBLIC
        Gnuradio::blocks
        signal_generator_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        algorithms_libs
)

target_include_directories(signal_generator_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_generator_adapters
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_generator_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_generator_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
