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

set(OBS_ADAPTER_SOURCES
    hybrid_observables.cc
)

set(OBS_ADAPTER_HEADERS
    hybrid_observables.h
)

source_group(Headers FILES ${OBS_ADAPTER_HEADERS})

add_library(obs_adapters ${OBS_ADAPTER_SOURCES} ${OBS_ADAPTER_HEADERS})

target_include_directories(obs_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

target_link_libraries(obs_adapters
    PUBLIC
        obs_gr_blocks
        gnss_rx
        gnss_sp_libs
    PRIVATE
        Gflags::gflags
        Glog::glog
)

set_property(TARGET obs_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
