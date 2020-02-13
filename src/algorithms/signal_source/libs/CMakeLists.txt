# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


if(ENABLE_FMCOMMS2 OR ENABLE_AD9361)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ad9361_manager.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ad9361_manager.h)
endif()

if(ENABLE_FPGA OR ENABLE_AD9361)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_switch.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_HEADERS} fpga_switch.h)
endif()

set(SIGNAL_SOURCE_LIB_SOURCES
    rtl_tcp_commands.cc
    rtl_tcp_dongle_info.cc
    gnss_sdr_valve.cc
    ${OPT_SIGNAL_SOURCE_LIB_SOURCES}
)

set(SIGNAL_SOURCE_LIB_HEADERS
    rtl_tcp_commands.h
    rtl_tcp_dongle_info.h
    gnss_sdr_valve.h
    ${OPT_SIGNAL_SOURCE_LIB_HEADERS}
)

list(SORT SIGNAL_SOURCE_LIB_HEADERS)
list(SORT SIGNAL_SOURCE_LIB_SOURCES)

source_group(Headers FILES ${SIGNAL_SOURCE_LIB_HEADERS})

add_library(signal_source_libs ${SIGNAL_SOURCE_LIB_SOURCES} ${SIGNAL_SOURCE_LIB_HEADERS})

target_link_libraries(signal_source_libs
    PUBLIC
        Boost::headers
        Gnuradio::runtime
    PRIVATE
        Gflags::gflags
        Glog::glog
        core_libs
)

target_include_directories(signal_source_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(ENABLE_FMCOMMS2 OR ENABLE_AD9361)
    target_link_libraries(signal_source_libs
        PUBLIC
            Iio::iio
        PRIVATE
            Iio::ad9361
        )
    if(LIBAD9361_VERSION)
        if(LIBAD9361_VERSION VERSION_GREATER 0.1)
            target_compile_definitions(signal_source_libs
                PRIVATE -DLIBAD9361_VERSION_GREATER_THAN_01=1
            )
        endif()
    endif()
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_source_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_source_libs APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
