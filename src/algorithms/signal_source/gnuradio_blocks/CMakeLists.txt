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


set(SIGNAL_SOURCE_GR_BLOCKS_SOURCES
     unpack_byte_2bit_samples.cc
     unpack_byte_2bit_cpx_samples.cc
     unpack_intspir_1bit_samples.cc
     rtl_tcp_signal_source_c.cc
     unpack_2bit_samples.cc
     unpack_spir_gss6450_samples.cc
     labsat23_source.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/algorithms/signal_source/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
)

file(GLOB SIGNAL_SOURCE_GR_BLOCKS_HEADERS "*.h")
list(SORT SIGNAL_SOURCE_GR_BLOCKS_HEADERS)
add_library(signal_source_gr_blocks ${SIGNAL_SOURCE_GR_BLOCKS_SOURCES} ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${SIGNAL_SOURCE_GR_BLOCKS_HEADERS})
target_link_libraries(signal_source_gr_blocks signal_source_lib ${GNURADIO_RUNTIME_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(signal_source_gr_blocks glog-${glog_RELEASE})
