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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

set(RESAMPLER_ADAPTER_SOURCES
        direct_resampler_conditioner.cc
        fractional_resampler_conditioner.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/algorithms/resampler/gnuradio_blocks
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_INCLUDE_DIRS}
)


if(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.11" )
     add_definitions( -DGR_GREATER_38=1 )
endif(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.11" )


file(GLOB RESAMPLER_ADAPTER_HEADERS "*.h")
list(SORT RESAMPLER_ADAPTER_HEADERS)
add_library(resampler_adapters ${RESAMPLER_ADAPTER_SOURCES} ${RESAMPLER_ADAPTER_HEADERS})
source_group(Headers FILES ${RESAMPLER_ADAPTER_HEADERS})
target_link_libraries(resampler_adapters resampler_gr_blocks)
