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

set(SIGNAL_GENERATOR_BLOCK_SOURCES signal_generator_c.cc)
set(SIGNAL_GENERATOR_BLOCK_HEADERS signal_generator_c.h)

source_group(Headers FILES ${SIGNAL_GENERATOR_BLOCK_HEADERS})

add_library(signal_generator_gr_blocks
    ${SIGNAL_GENERATOR_BLOCK_SOURCES}
    ${SIGNAL_GENERATOR_BLOCK_HEADERS}
)

target_link_libraries(signal_generator_gr_blocks
    PUBLIC
        Gnuradio::runtime
        core_system_parameters
    PRIVATE
        Volkgnsssdr::volkgnsssdr
        algorithms_libs
)

target_include_directories(signal_generator_gr_blocks
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_generator_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_generator_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
