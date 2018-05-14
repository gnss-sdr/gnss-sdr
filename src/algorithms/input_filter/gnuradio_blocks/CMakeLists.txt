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


set(INPUT_FILTER_GR_BLOCKS_SOURCES
     beamformer.cc
     pulse_blanking_cc.cc
     notch_cc.cc
     notch_lite_cc.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${GNURADIO_BLOCKS_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
)

file(GLOB INPUT_FILTER_GR_BLOCKS_HEADERS "*.h")
list(SORT INPUT_FILTER_GR_BLOCKS_HEADERS)
add_library(input_filter_gr_blocks ${INPUT_FILTER_GR_BLOCKS_SOURCES} ${INPUT_FILTER_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${INPUT_FILTER_GR_BLOCKS_HEADERS})

target_link_libraries(input_filter_gr_blocks ${GNURADIO_FILTER_LIBRARIES}  ${VOLK_GNSSSDR_LIBRARIES} ${LOG4CPP_LIBRARIES})

if(NOT VOLK_GNSSSDR_FOUND)
    add_dependencies(input_filter_gr_blocks volk_gnsssdr_module glog-${glog_RELEASE})
else(NOT VOLK_GNSSSDR_FOUND)
    add_dependencies(input_filter_gr_blocks glog-${glog_RELEASE})
endif(NOT VOLK_GNSSSDR_FOUND)
