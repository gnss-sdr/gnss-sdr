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


set(COND_ADAPTER_SOURCES
    signal_conditioner.cc
    array_signal_conditioner.cc
)

set(COND_ADAPTER_HEADERS
    signal_conditioner.h
    array_signal_conditioner.h
)

list(SORT COND_ADAPTER_HEADERS)
list(SORT COND_ADAPTER_SOURCES)

source_group(Headers FILES ${COND_ADAPTER_HEADERS})

add_library(conditioner_adapters ${COND_ADAPTER_SOURCES} ${COND_ADAPTER_HEADERS})

target_link_libraries(conditioner_adapters
    PUBLIC
        Gnuradio::runtime
    PRIVATE
        Gflags::gflags
        Glog::glog
)

target_include_directories(conditioner_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(conditioner_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET conditioner_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
