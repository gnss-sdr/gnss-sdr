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


set(DATATYPE_ADAPTER_SOURCES
    byte_to_short.cc
    ibyte_to_cbyte.cc
    ibyte_to_complex.cc
    ibyte_to_cshort.cc
    ishort_to_cshort.cc
    ishort_to_complex.cc
)

set(DATATYPE_ADAPTER_HEADERS
    byte_to_short.h
    ibyte_to_cbyte.h
    ibyte_to_complex.h
    ibyte_to_cshort.h
    ishort_to_cshort.h
    ishort_to_complex.h
)

list(SORT DATATYPE_ADAPTER_HEADERS)
list(SORT DATATYPE_ADAPTER_SOURCES)

source_group(Headers FILES ${DATATYPE_ADAPTER_HEADERS})

add_library(data_type_adapters
    ${DATATYPE_ADAPTER_SOURCES}
    ${DATATYPE_ADAPTER_HEADERS}
)

target_link_libraries(data_type_adapters
    PUBLIC
        Gnuradio::blocks
        data_type_gr_blocks
        algorithms_libs
        core_system_parameters
    PRIVATE
        Gflags::gflags
        Glog::glog
        Volk::volk
)

target_include_directories(data_type_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/algorithms/libs
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(data_type_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET data_type_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
