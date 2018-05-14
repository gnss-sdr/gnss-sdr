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

set(OBS_ADAPTER_SOURCES
	hybrid_observables.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/observables/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
)

file(GLOB OBS_ADAPTER_HEADERS "*.h")
list(SORT OBS_ADAPTER_HEADERS)
add_library(obs_adapters ${OBS_ADAPTER_SOURCES} ${OBS_ADAPTER_HEADERS})
source_group(Headers FILES ${OBS_ADAPTER_HEADERS})
target_link_libraries(obs_adapters obs_gr_blocks ${GNURADIO_RUNTIME_LIBRARIES})
