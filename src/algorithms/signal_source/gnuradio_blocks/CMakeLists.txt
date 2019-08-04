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


if(ENABLE_RAW_UDP AND PCAP_FOUND)
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} gr_complex_ip_packet_source.cc)
    set(OPT_DRIVER_HEADERS ${OPT_DRIVER_HEADERS} gr_complex_ip_packet_source.h)
endif()


set(SIGNAL_SOURCE_GR_BLOCKS_SOURCES
    unpack_byte_2bit_samples.cc
    unpack_byte_2bit_cpx_samples.cc
    unpack_byte_4bit_samples.cc
    unpack_intspir_1bit_samples.cc
    rtl_tcp_signal_source_c.cc
    unpack_2bit_samples.cc
    unpack_spir_gss6450_samples.cc
    labsat23_source.cc
    ${OPT_DRIVER_SOURCES}
)


set(SIGNAL_SOURCE_GR_BLOCKS_HEADERS
    unpack_byte_2bit_samples.h
    unpack_byte_2bit_cpx_samples.h
    unpack_byte_4bit_samples.h
    unpack_intspir_1bit_samples.h
    rtl_tcp_signal_source_c.h
    unpack_2bit_samples.h
    unpack_spir_gss6450_samples.h
    labsat23_source.h
    ${OPT_DRIVER_HEADERS}
)

list(SORT SIGNAL_SOURCE_GR_BLOCKS_HEADERS)
list(SORT SIGNAL_SOURCE_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})

add_library(signal_source_gr_blocks
    ${SIGNAL_SOURCE_GR_BLOCKS_SOURCES}
    ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS}
)

target_link_libraries(signal_source_gr_blocks
    PUBLIC
        signal_source_libs
        Boost::thread
        Gnuradio::runtime
    PRIVATE
        core_libs
        Gflags::gflags
        Glog::glog
)

target_include_directories(signal_source_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(ENABLE_RAW_UDP AND PCAP_FOUND)
    target_link_libraries(signal_source_gr_blocks
        PUBLIC
            Pcap::pcap
    )
endif()

# Fix for Boost Asio < 1.70
if(OS_IS_MACOSX)
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(signal_source_gr_blocks
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(signal_source_gr_blocks
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.65.99)
    target_compile_definitions(signal_source_gr_blocks
        PUBLIC
            -DBOOST_GREATER_1_65
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_source_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_source_gr_blocks APPEND PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
