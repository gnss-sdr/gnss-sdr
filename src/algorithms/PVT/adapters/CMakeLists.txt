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


if(Boost_VERSION LESS 105800)
    add_definitions(-DOLD_BOOST=1)
endif(Boost_VERSION LESS 105800)

set(PVT_ADAPTER_SOURCES
    rtklib_pvt.cc
)

set(PVT_ADAPTER_HEADERS
    rtklib_pvt.h
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${CMAKE_SOURCE_DIR}/src/core/interfaces
    ${CMAKE_SOURCE_DIR}/src/core/receiver
    ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/gnuradio_blocks
    ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs/rtklib
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs
    ${ARMADILLO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
)

add_library(pvt_adapters ${PVT_ADAPTER_SOURCES} ${PVT_ADAPTER_HEADERS})

source_group(Headers FILES ${PVT_ADAPTER_HEADERS})

target_link_libraries(pvt_adapters
    pvt_gr_blocks
    ${ARMADILLO_LIBRARIES}
    ${GNURADIO_RUNTIME_LIBRARIES}
)
