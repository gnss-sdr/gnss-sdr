# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

set(INPUT_FILTER_ADAPTER_SOURCES 
     fir_filter.cc
     freq_xlating_fir_filter.cc
     beamformer_filter.cc
     pulse_blanking_filter.cc
     notch_filter.cc
     notch_filter_lite.cc
)

set(INPUT_FILTER_ADAPTER_HEADERS 
     fir_filter.h
     freq_xlating_fir_filter.h
     beamformer_filter.h
     pulse_blanking_filter.h
     notch_filter.h
     notch_filter_lite.h
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/algorithms/input_filter/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
)

if(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13.4" )
     add_definitions( -DGR_GREATER_38=1 )
endif(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13.4" )

list(SORT INPUT_FILTER_ADAPTER_HEADERS)
list(SORT INPUT_FILTER_ADAPTER_SOURCES)

add_library(input_filter_adapters ${INPUT_FILTER_ADAPTER_SOURCES} ${INPUT_FILTER_ADAPTER_HEADERS})
source_group(Headers FILES ${INPUT_FILTER_ADAPTER_HEADERS})
add_dependencies(input_filter_adapters glog-${glog_RELEASE} gnss_sp_libs)
target_link_libraries(input_filter_adapters input_filter_gr_blocks ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES} ${GNURADIO_FILTER_LIBRARIES} gnss_sp_libs)
