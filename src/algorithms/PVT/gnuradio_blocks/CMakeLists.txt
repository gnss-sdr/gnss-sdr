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

set(PVT_GR_BLOCKS_SOURCES
    rtklib_pvt_gs.cc
)

set(PVT_GR_BLOCKS_HEADERS
    rtklib_pvt_gs.h
)

source_group(Headers FILES ${PVT_GR_BLOCKS_HEADERS})

add_library(pvt_gr_blocks ${PVT_GR_BLOCKS_SOURCES} ${PVT_GR_BLOCKS_HEADERS})

if(${FILESYSTEM_FOUND})
    target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(pvt_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(pvt_gr_blocks PRIVATE Boost::filesystem Boost::system)
endif()

target_link_libraries(pvt_gr_blocks
    PUBLIC
        algorithms_libs_rtklib
        core_system_parameters
        Boost::date_time
        Gnuradio::pmt
        Gnuradio::runtime
    PRIVATE
        pvt_libs
        algorithms_libs
        Gflags::gflags
        Glog::glog
        Boost::serialization
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(pvt_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

if(Boost_VERSION_STRING VERSION_LESS 1.58.0)
    target_compile_definitions(pvt_gr_blocks PRIVATE -DOLD_BOOST=1)
endif()

# Check if we have std::put_time (Workaround for gcc < 5.0)
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
    #include <iomanip>
    int main()
    { std::put_time(nullptr, \"\"); }"
    has_put_time
)
if(${has_put_time})
    target_compile_definitions(pvt_gr_blocks PRIVATE -DHAS_PUT_TIME=1)
endif()

set_property(TARGET pvt_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
