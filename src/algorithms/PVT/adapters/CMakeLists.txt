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


set(PVT_ADAPTER_SOURCES
    rtklib_pvt.cc
)

set(PVT_ADAPTER_HEADERS
    rtklib_pvt.h
)

source_group(Headers FILES ${PVT_ADAPTER_HEADERS})

add_library(pvt_adapters ${PVT_ADAPTER_SOURCES} ${PVT_ADAPTER_HEADERS})

target_link_libraries(pvt_adapters
    PUBLIC
        pvt_gr_blocks
        algorithms_libs_rtklib
        core_system_parameters
    PRIVATE
        gnss_sdr_flags
        pvt_libs
)

target_include_directories(pvt_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(Boost_VERSION_STRING VERSION_LESS 1.58.0)
    target_compile_definitions(pvt_adapters PRIVATE -DOLD_BOOST=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET pvt_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
