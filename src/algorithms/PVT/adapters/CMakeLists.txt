# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


set(PVT_ADAPTER_SOURCES
    rtklib_pvt.cc
)

set(PVT_ADAPTER_HEADERS
    rtklib_pvt.h
)

source_group(Headers FILES ${PVT_ADAPTER_HEADERS})

add_library(pvt_adapters ${PVT_ADAPTER_SOURCES} ${PVT_ADAPTER_HEADERS})

target_link_libraries(pvt_adapters
    PUBLIC
        pvt_gr_blocks
        algorithms_libs_rtklib
    PRIVATE
        gnss_sdr_flags
        pvt_libs
)

target_include_directories(pvt_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(Boost_VERSION_STRING VERSION_LESS 1.58.0)
    target_compile_definitions(pvt_adapters PRIVATE -DOLD_BOOST=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET pvt_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
