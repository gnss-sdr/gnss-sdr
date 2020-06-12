# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


if(USE_CMAKE_TARGET_SOURCES)
    add_library(pvt_adapters STATIC)
    target_sources(pvt_adapters
        PRIVATE
            rtklib_pvt.cc
        PUBLIC
            rtklib_pvt.h
    )
else()
    source_group(Headers FILES rtklib_pvt.h)
    add_library(pvt_adapters rtklib_pvt.cc rtklib_pvt.h)
endif()

target_link_libraries(pvt_adapters
    PUBLIC
        pvt_gr_blocks
    PRIVATE
        gnss_sdr_flags
        Glog::glog
        pvt_libs
)

target_include_directories(pvt_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(USE_OLD_BOOST_MATH_COMMON_FACTOR)
    target_compile_definitions(pvt_adapters
        PRIVATE
            -DUSE_OLD_BOOST_MATH_COMMON_FACTOR=1
    )
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
