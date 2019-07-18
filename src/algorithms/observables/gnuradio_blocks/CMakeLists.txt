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
    hybrid_observables_gs.cc
)

set(OBS_GR_BLOCKS_HEADERS
    hybrid_observables_gs.h
)

source_group(Headers FILES ${OBS_GR_BLOCKS_HEADERS})

add_library(obs_gr_blocks ${OBS_GR_BLOCKS_SOURCES} ${OBS_GR_BLOCKS_HEADERS})

if(${FILESYSTEM_FOUND})
    target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(obs_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(obs_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(obs_gr_blocks PRIVATE Boost::filesystem)
endif()

target_include_directories(obs_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

target_link_libraries(obs_gr_blocks
    PUBLIC
        Boost::boost
        Gnuradio::blocks
    PRIVATE
        algorithms_libs
        core_system_parameters
        Gflags::gflags
        Glog::glog
        Matio::matio
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obs_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET obs_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
