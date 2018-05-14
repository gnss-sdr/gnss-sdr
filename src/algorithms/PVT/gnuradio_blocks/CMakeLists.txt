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

set(PVT_GR_BLOCKS_SOURCES
	rtklib_pvt_cc.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs/rtklib
     ${ARMADILLO_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
)

file(GLOB PVT_GR_BLOCKS_HEADERS "*.h")
list(SORT PVT_GR_BLOCKS_HEADERS)
add_library(pvt_gr_blocks ${PVT_GR_BLOCKS_SOURCES} ${PVT_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${PVT_GR_BLOCKS_HEADERS})
target_link_libraries(pvt_gr_blocks pvt_lib ${ARMADILLO_LIBRARIES})
