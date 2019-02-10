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

set(PVT_GR_BLOCKS_SOURCES
    rtklib_pvt_cc.cc
)

set(PVT_GR_BLOCKS_HEADERS
    rtklib_pvt_cc.h
)

source_group(Headers FILES ${PVT_GR_BLOCKS_HEADERS})

add_library(pvt_gr_blocks ${PVT_GR_BLOCKS_SOURCES} ${PVT_GR_BLOCKS_HEADERS})

target_link_libraries(pvt_gr_blocks
    PUBLIC
        Boost::date_time
        Gnuradio::runtime
        pvt_lib
    PRIVATE
        Gflags::gflags
        Glog::glog
        Boost::filesystem
        Boost::system
        gnss_system_parameters
)

target_include_directories(pvt_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/libs
)

if(Boost_VERSION LESS 105800)
    target_compile_definitions(pvt_gr_blocks PRIVATE -DOLD_BOOST=1)
endif()

set_property(TARGET pvt_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core/system_parameters>
)
