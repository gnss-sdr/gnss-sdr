# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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
    geofunctions.cc
    spirent_motion_csv_dump_reader.cc
    rtklib_solver_dump_reader.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${MATIO_INCLUDE_DIRS}
)


file(GLOB SYSTEM_TESTING_LIB_HEADERS "*.h")
list(SORT SYSTEM_TESTING_LIB_HEADERS)
add_library(system_testing_lib ${SYSTEM_TESTING_LIB_SOURCES} ${SYSTEM_TESTING_LIB_HEADERS})
source_group(Headers FILES ${SYSTEM_TESTING_LIB_HEADERS})

if(NOT MATIO_FOUND)
   add_dependencies(system_testing_lib matio-${GNSSSDR_MATIO_LOCAL_VERSION})
endif(NOT MATIO_FOUND)