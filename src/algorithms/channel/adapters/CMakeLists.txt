# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_library(channel_adapters STATIC)
    target_sources(channel_adapters
        PRIVATE
            channel.cc
        PUBLIC
            channel.h
    )
else()
    source_group(Headers FILES channel.h)
    add_library(channel_adapters
        channel.cc
        channel.h
    )
endif()

target_link_libraries(channel_adapters
    PUBLIC
        channel_libs
    PRIVATE
        gnss_sdr_flags
        Glog::glog
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
