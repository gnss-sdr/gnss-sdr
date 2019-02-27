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


set(RESAMPLER_GR_BLOCKS_SOURCES
    direct_resampler_conditioner_cc.cc
    direct_resampler_conditioner_cs.cc
    direct_resampler_conditioner_cb.cc
)

set(RESAMPLER_GR_BLOCKS_HEADERS
    direct_resampler_conditioner_cc.h
    direct_resampler_conditioner_cs.h
    direct_resampler_conditioner_cb.h
)

list(SORT RESAMPLER_GR_BLOCKS_HEADERS)
list(SORT RESAMPLER_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${RESAMPLER_GR_BLOCKS_HEADERS})

add_library(resampler_gr_blocks
    ${RESAMPLER_GR_BLOCKS_SOURCES}
    ${RESAMPLER_GR_BLOCKS_HEADERS}
)

target_link_libraries(resampler_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Volk::volk
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(resampler_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET resampler_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
