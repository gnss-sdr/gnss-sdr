# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
#

if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
    if(NOT GRIIO_FOUND)
        message(STATUS "gnuradio-iio not found, its installation is required.")
        message(STATUS "Please build and install the following projects:")
        message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
        message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
        message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
        message(FATAL_ERROR "gnuradio-iio is required for building gnss-sdr with this option enabled")
    endif()
endif()

if(ENABLE_FMCOMMS2 OR ENABLE_AD9361)
    if(NOT LIBIIO_FOUND)
        message(STATUS "libiio not found, its installation is required.")
        message(STATUS "Please build and install the following projects:")
        message(STATUS " * libiio from https://github.com/analogdevicesinc/libiio")
        message(STATUS " * libad9361-iio from https://github.com/analogdevicesinc/libad9361-iio")
        message(STATUS " * gnuradio-iio from https://github.com/analogdevicesinc/gr-iio")
        message(FATAL_ERROR "libiio is required for building gnss-sdr with this option enabled")
    else()
        set(OPT_SIGNAL_SOURCE_LIB_SOURCES ad9361_manager.cc)
        set(OPT_SIGNAL_SOURCE_LIB_HEADERS ad9361_manager.h)
    endif()
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
        Boost::boost
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

if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
    target_link_libraries(signal_source_libs
        PUBLIC
            Gnuradio::iio
        PRIVATE
            Iio::iio
    )
endif()

if(ENABLE_FMCOMMS2 OR ENABLE_AD9361)
    target_link_libraries(signal_source_libs
        PUBLIC
            Iio::iio
    )
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
