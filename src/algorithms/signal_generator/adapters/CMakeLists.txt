# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
    add_library(signal_generator_adapters STATIC)
    target_sources(signal_generator_adapters
        PRIVATE
            signal_generator.cc
        PUBLIC
            signal_generator.h
    )
else()
    source_group(Headers FILES signal_generator.h)
    add_library(signal_generator_adapters
        signal_generator.cc
        signal_generator.h
    )
endif()

target_link_libraries(signal_generator_adapters
    PUBLIC
        Gnuradio::blocks
        Gnuradio::pmt
        signal_generator_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        core_system_parameters
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

if(GNURADIO_USES_SPDLOG)
    target_link_libraries(signal_generator_adapters
        PUBLIC
            fmt::fmt
            spdlog::spdlog
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
