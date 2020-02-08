# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(CHANNEL_ADAPTER_SOURCES channel.cc)
set(CHANNEL_ADAPTER_HEADERS channel.h)

add_library(channel_adapters
    ${CHANNEL_ADAPTER_SOURCES}
    ${CHANNEL_ADAPTER_HEADERS}
)

source_group(Headers FILES ${CHANNEL_ADAPTER_HEADERS})

target_link_libraries(channel_adapters
    PUBLIC
        Gnuradio::runtime
        channel_libs
        core_system_parameters
    PRIVATE
        Gflags::gflags
        Glog::glog
        gnss_sdr_flags
)

target_include_directories(channel_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(channel_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET channel_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
