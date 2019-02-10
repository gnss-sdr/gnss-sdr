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

set(OBS_GR_BLOCKS_SOURCES
    hybrid_observables_cc.cc
)

set(OBS_GR_BLOCKS_HEADERS
    hybrid_observables_cc.h
)

source_group(Headers FILES ${OBS_GR_BLOCKS_HEADERS})

add_library(obs_gr_blocks ${OBS_GR_BLOCKS_SOURCES} ${OBS_GR_BLOCKS_HEADERS})

target_include_directories(obs_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

target_link_libraries(obs_gr_blocks
    PUBLIC
        Boost::boost
        Gnuradio::blocks
        core_system_parameters
    PRIVATE
        Gflags::gflags
        Glog::glog
        Boost::filesystem
        Matio::matio
        algorithms_libs
)

set_property(TARGET obs_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
