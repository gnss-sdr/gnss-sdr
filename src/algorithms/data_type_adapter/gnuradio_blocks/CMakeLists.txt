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


set(DATA_TYPE_GR_BLOCKS_SOURCES
    interleaved_byte_to_complex_byte.cc
    interleaved_short_to_complex_short.cc
    interleaved_byte_to_complex_short.cc
)

set(DATA_TYPE_GR_BLOCKS_HEADERS
    interleaved_byte_to_complex_byte.h
    interleaved_short_to_complex_short.h
    interleaved_byte_to_complex_short.h
)

list(SORT DATA_TYPE_GR_BLOCKS_HEADERS)
list(SORT DATA_TYPE_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${DATA_TYPE_GR_BLOCKS_HEADERS})

add_library(data_type_gr_blocks
    ${DATA_TYPE_GR_BLOCKS_SOURCES}
    ${DATA_TYPE_GR_BLOCKS_HEADERS}
)

target_link_libraries(data_type_gr_blocks
    PUBLIC
        Gnuradio::runtime
        Boost::boost
        algorithms_libs
    PRIVATE
        Volk::volk
)

set_property(TARGET data_type_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
