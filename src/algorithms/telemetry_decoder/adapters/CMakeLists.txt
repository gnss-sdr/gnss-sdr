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


set(TELEMETRY_DECODER_ADAPTER_SOURCES
    gps_l1_ca_telemetry_decoder.cc
    gps_l2c_telemetry_decoder.cc
    gps_l5_telemetry_decoder.cc
    galileo_e1b_telemetry_decoder.cc
    sbas_l1_telemetry_decoder.cc
    galileo_e5a_telemetry_decoder.cc
    glonass_l1_ca_telemetry_decoder.cc
    glonass_l2_ca_telemetry_decoder.cc
    beidou_b1i_telemetry_decoder.cc
    beidou_b3i_telemetry_decoder.cc
)

set(TELEMETRY_DECODER_ADAPTER_HEADERS
    gps_l1_ca_telemetry_decoder.h
    gps_l2c_telemetry_decoder.h
    gps_l5_telemetry_decoder.h
    galileo_e1b_telemetry_decoder.h
    sbas_l1_telemetry_decoder.h
    galileo_e5a_telemetry_decoder.h
    glonass_l1_ca_telemetry_decoder.h
    glonass_l2_ca_telemetry_decoder.h
    beidou_b1i_telemetry_decoder.h
    beidou_b3i_telemetry_decoder.h 
)

list(SORT TELEMETRY_DECODER_ADAPTER_HEADERS)
list(SORT TELEMETRY_DECODER_ADAPTER_SOURCES)

source_group(Headers FILES ${TELEMETRY_DECODER_ADAPTER_HEADERS})

add_library(telemetry_decoder_adapters
    ${TELEMETRY_DECODER_ADAPTER_SOURCES}
    ${TELEMETRY_DECODER_ADAPTER_HEADERS}
)

target_link_libraries(telemetry_decoder_adapters
    PUBLIC
        telemetry_decoder_gr_blocks
        core_system_parameters
    PRIVATE
        Gflags::gflags
        Glog::glog
        Gnuradio::runtime
        telemetry_decoder_libs
)

target_include_directories(telemetry_decoder_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
