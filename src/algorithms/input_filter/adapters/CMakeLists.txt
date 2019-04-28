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

set(INPUT_FILTER_ADAPTER_SOURCES
    fir_filter.cc
    freq_xlating_fir_filter.cc
    beamformer_filter.cc
    pulse_blanking_filter.cc
    notch_filter.cc
    notch_filter_lite.cc
)

set(INPUT_FILTER_ADAPTER_HEADERS
    fir_filter.h
    freq_xlating_fir_filter.h
    beamformer_filter.h
    pulse_blanking_filter.h
    notch_filter.h
    notch_filter_lite.h
)

list(SORT INPUT_FILTER_ADAPTER_HEADERS)
list(SORT INPUT_FILTER_ADAPTER_SOURCES)

source_group(Headers FILES ${INPUT_FILTER_ADAPTER_HEADERS})

add_library(input_filter_adapters
    ${INPUT_FILTER_ADAPTER_SOURCES}
    ${INPUT_FILTER_ADAPTER_HEADERS}
)

target_link_libraries(input_filter_adapters
    PUBLIC
        Gnuradio::blocks
        Gnuradio::filter
        input_filter_gr_blocks
        algorithms_libs
    PRIVATE
        Gflags::gflags
        Glog::glog
        Volk::volk
)

target_include_directories(input_filter_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(NOT (GNURADIO_VERSION VERSION_LESS "3.8"))
    target_compile_definitions(input_filter_adapters PUBLIC -DGR_GREATER_38=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(input_filter_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET input_filter_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
