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

set(OBS_GR_BLOCKS_SOURCES 
	hybrid_observables_cc.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${MATIO_INCLUDE_DIRS}
)

file(GLOB OBS_GR_BLOCKS_HEADERS "*.h")
list(SORT OBS_GR_BLOCKS_HEADERS)
add_library(obs_gr_blocks ${OBS_GR_BLOCKS_SOURCES} ${OBS_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${OBS_GR_BLOCKS_HEADERS})
if(MATIO_FOUND)
    add_dependencies(obs_gr_blocks glog-${glog_RELEASE} armadillo-${armadillo_RELEASE})
else(MATIO_FOUND)
    add_dependencies(obs_gr_blocks glog-${glog_RELEASE} armadillo-${armadillo_RELEASE} matio-${GNSSSDR_MATIO_LOCAL_VERSION})
endif(MATIO_FOUND)
target_link_libraries(obs_gr_blocks ${GNURADIO_RUNTIME_LIBRARIES} ${ARMADILLO_LIBRARIES} ${MATIO_LIBRARIES})
