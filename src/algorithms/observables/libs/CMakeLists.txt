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

set(OBSERVABLES_LIB_HEADERS ${OBSERVABLES_LIB_HEADERS} obs_conf.h)
set(OBSERVABLES_LIB_SOURCES ${OBSERVABLES_LIB_SOURCES} obs_conf.cc)

list(SORT OBSERVABLES_LIB_HEADERS)
list(SORT OBSERVABLES_LIB_SOURCES)

source_group(Headers FILES ${OBSERVABLES_LIB_HEADERS})

add_library(observables_libs
    ${OBSERVABLES_LIB_SOURCES}
    ${OBSERVABLES_LIB_HEADERS}
)

target_link_libraries(observables_libs
    PRIVATE
        Gflags::gflags
        Glog::glog
        algorithms_libs
        core_system_parameters
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(observables_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET observables_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
