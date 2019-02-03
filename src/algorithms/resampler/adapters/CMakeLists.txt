# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
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


set(RESAMPLER_ADAPTER_SOURCES
    direct_resampler_conditioner.cc
    mmse_resampler_conditioner.cc
)

set(RESAMPLER_ADAPTER_HEADERS
    direct_resampler_conditioner.h
    mmse_resampler_conditioner.h
)

list(SORT RESAMPLER_ADAPTER_HEADERS)
list(SORT RESAMPLER_ADAPTER_SOURCES)

add_library(resampler_adapters
    ${RESAMPLER_ADAPTER_SOURCES}
    ${RESAMPLER_ADAPTER_HEADERS}
)

source_group(Headers FILES ${RESAMPLER_ADAPTER_HEADERS})

target_link_libraries(resampler_adapters
    PUBLIC
        Gnuradio::runtime
        resampler_gr_blocks
        gnss_rx
    PRIVATE
        Glog::glog
        Volk::volk
)

if(${PC_GNURADIO_RUNTIME_VERSION} VERSION_GREATER "3.7.13.4")
    target_compile_definitions(resampler_adapters PUBLIC -DGR_GREATER_38=1)
endif()

set_property(TARGET resampler_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
