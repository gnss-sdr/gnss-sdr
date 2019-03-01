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


set(SYSTEM_TESTING_LIB_SOURCES
    spirent_motion_csv_dump_reader.cc
    rtklib_solver_dump_reader.cc
)

file(GLOB SYSTEM_TESTING_LIB_HEADERS "*.h")
list(SORT SYSTEM_TESTING_LIB_HEADERS)

add_library(system_testing_lib
    ${SYSTEM_TESTING_LIB_SOURCES}
    ${SYSTEM_TESTING_LIB_HEADERS}
)

source_group(Headers FILES ${SYSTEM_TESTING_LIB_HEADERS})

target_link_libraries(system_testing_lib
    PUBLIC
        Armadillo::armadillo
        Gflags::gflags
    PRIVATE
        Boost::boost
        Matio::matio
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(system_testing_lib
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET system_testing_lib
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
