# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(SIGNAL_GENERATOR_BLOCK_SOURCES signal_generator_c.cc)
set(SIGNAL_GENERATOR_BLOCK_HEADERS signal_generator_c.h)

source_group(Headers FILES ${SIGNAL_GENERATOR_BLOCK_HEADERS})

add_library(signal_generator_gr_blocks
    ${SIGNAL_GENERATOR_BLOCK_SOURCES}
    ${SIGNAL_GENERATOR_BLOCK_HEADERS}
)

target_link_libraries(signal_generator_gr_blocks
    PUBLIC
        Gnuradio::runtime
        core_system_parameters
    PRIVATE
        Volkgnsssdr::volkgnsssdr
        algorithms_libs
)

target_include_directories(signal_generator_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_generator_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_generator_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_generator_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
