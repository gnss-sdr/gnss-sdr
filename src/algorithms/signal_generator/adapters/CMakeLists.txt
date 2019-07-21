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

set(SIGNAL_GENERATOR_ADAPTER_SOURCES signal_generator.cc)
set(SIGNAL_GENERATOR_ADAPTER_HEADERS signal_generator.h)

source_group(Headers FILES ${SIGNAL_GENERATOR_ADAPTER_HEADERS})

add_library(signal_generator_adapters
    ${SIGNAL_GENERATOR_ADAPTER_SOURCES}
    ${SIGNAL_GENERATOR_ADAPTER_HEADERS}
)

target_link_libraries(signal_generator_adapters
    PUBLIC
        Gnuradio::blocks
        signal_generator_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        algorithms_libs
)

target_include_directories(signal_generator_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_generator_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_generator_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
