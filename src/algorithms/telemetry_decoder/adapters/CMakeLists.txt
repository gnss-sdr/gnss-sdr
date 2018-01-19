# Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

set(TELEMETRY_DECODER_ADAPTER_SOURCES 
    gps_l1_ca_telemetry_decoder.cc 
    gps_l2c_telemetry_decoder.cc
    gps_l5_telemetry_decoder.cc 
    galileo_e1b_telemetry_decoder.cc
    sbas_l1_telemetry_decoder.cc
    galileo_e5a_telemetry_decoder.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs/libswiftcnav
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
)

file(GLOB TELEMETRY_DECODER_ADAPTER_HEADERS "*.h")
list(SORT TELEMETRY_DECODER_ADAPTER_HEADERS)
add_library(telemetry_decoder_adapters ${TELEMETRY_DECODER_ADAPTER_SOURCES} ${TELEMETRY_DECODER_ADAPTER_HEADERS})
source_group(Headers FILES ${TELEMETRY_DECODER_ADAPTER_HEADERS})
target_link_libraries(telemetry_decoder_adapters telemetry_decoder_gr_blocks gnss_system_parameters ${GNURADIO_RUNTIME_LIBRARIES})
