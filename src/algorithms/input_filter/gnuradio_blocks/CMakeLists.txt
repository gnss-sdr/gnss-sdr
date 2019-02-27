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


set(INPUT_FILTER_GR_BLOCKS_SOURCES
    beamformer.cc
    pulse_blanking_cc.cc
    notch_cc.cc
    notch_lite_cc.cc
)

set(INPUT_FILTER_GR_BLOCKS_HEADERS
    beamformer.h
    pulse_blanking_cc.h
    notch_cc.h
    notch_lite_cc.h
)

list(SORT INPUT_FILTER_GR_BLOCKS_HEADERS)
list(SORT INPUT_FILTER_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${INPUT_FILTER_GR_BLOCKS_HEADERS})

add_library(input_filter_gr_blocks
    ${INPUT_FILTER_GR_BLOCKS_SOURCES}
    ${INPUT_FILTER_GR_BLOCKS_HEADERS})

target_link_libraries(input_filter_gr_blocks
    PUBLIC
        Gnuradio::blocks
        Gnuradio::filter
        Volkgnsssdr::volkgnsssdr
    PRIVATE
        Log4cpp::log4cpp
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(input_filter_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET input_filter_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
