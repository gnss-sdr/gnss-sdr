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

set(TELEMETRY_DECODER_GR_BLOCKS_SOURCES
    gps_l1_ca_telemetry_decoder_gs.cc
    gps_l2c_telemetry_decoder_gs.cc
    gps_l5_telemetry_decoder_gs.cc
    sbas_l1_telemetry_decoder_gs.cc
    glonass_l1_ca_telemetry_decoder_gs.cc
    glonass_l2_ca_telemetry_decoder_gs.cc
    galileo_telemetry_decoder_gs.cc
    beidou_b1i_telemetry_decoder_gs.cc
    beidou_b3i_telemetry_decoder_gs.cc
)

set(TELEMETRY_DECODER_GR_BLOCKS_HEADERS
    gps_l1_ca_telemetry_decoder_gs.h
    gps_l2c_telemetry_decoder_gs.h
    gps_l5_telemetry_decoder_gs.h
    sbas_l1_telemetry_decoder_gs.h
    glonass_l1_ca_telemetry_decoder_gs.h
    glonass_l2_ca_telemetry_decoder_gs.h
    galileo_telemetry_decoder_gs.h
    beidou_b1i_telemetry_decoder_gs.h
    beidou_b3i_telemetry_decoder_gs.h
)

list(SORT TELEMETRY_DECODER_GR_BLOCKS_HEADERS)
list(SORT TELEMETRY_DECODER_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS})

add_library(telemetry_decoder_gr_blocks
    ${TELEMETRY_DECODER_GR_BLOCKS_SOURCES}
    ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS}
)

target_link_libraries(telemetry_decoder_gr_blocks
    PUBLIC
        telemetry_decoder_libswiftcnav
        telemetry_decoder_libs
        core_system_parameters
        Gnuradio::runtime
        Boost::boost
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
