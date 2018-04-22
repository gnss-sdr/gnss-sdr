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

set(TELEMETRY_DECODER_GR_BLOCKS_SOURCES
     gps_l1_ca_telemetry_decoder_cc.cc
     gps_l2c_telemetry_decoder_cc.cc
     gps_l5_telemetry_decoder_cc.cc
     galileo_e1b_telemetry_decoder_cc.cc
     sbas_l1_telemetry_decoder_cc.cc
     galileo_e5a_telemetry_decoder_cc.cc
     glonass_l1_ca_telemetry_decoder_cc.cc
     glonass_l2_ca_telemetry_decoder_cc.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/telemetry_decoder/libs/libswiftcnav
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

file(GLOB TELEMETRY_DECODER_GR_BLOCKS_HEADERS "*.h")
list(SORT TELEMETRY_DECODER_GR_BLOCKS_HEADERS)
add_library(telemetry_decoder_gr_blocks ${TELEMETRY_DECODER_GR_BLOCKS_SOURCES} ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${TELEMETRY_DECODER_GR_BLOCKS_HEADERS})
target_link_libraries(telemetry_decoder_gr_blocks telemetry_decoder_libswiftcnav telemetry_decoder_lib gnss_system_parameters ${GNURADIO_RUNTIME_LIBRARIES} ${VOLK_GNSSSDR_LIBRARIES})

if(NOT VOLK_GNSSSDR_FOUND)
     add_dependencies(telemetry_decoder_gr_blocks volk_gnsssdr_module)
endif(NOT VOLK_GNSSSDR_FOUND)
