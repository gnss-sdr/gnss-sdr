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

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src/core/system_parameters
    ${CMAKE_SOURCE_DIR}/src/core/interfaces
    ${CMAKE_SOURCE_DIR}/src/algorithms/data_type_adapter/gnuradio_blocks
    ${CMAKE_SOURCE_DIR}/src/algorithms/libs
    ${GLOG_INCLUDE_DIRS}
    ${GFlags_INCLUDE_DIRS}
    ${GNURADIO_RUNTIME_INCLUDE_DIRS}
    ${VOLK_INCLUDE_DIRS}
)

list(SORT DATATYPE_ADAPTER_HEADERS)
list(SORT DATATYPE_ADAPTER_SOURCES)

add_library(datatype_adapters
    ${DATATYPE_ADAPTER_SOURCES}
    ${DATATYPE_ADAPTER_HEADERS}
)

source_group(Headers FILES ${DATATYPE_ADAPTER_HEADERS})
add_dependencies(datatype_adapters glog-${glog_RELEASE})
target_link_libraries(datatype_adapters
    data_type_gr_blocks
    ${GNURADIO_RUNTIME_LIBRARIES}
    ${GNURADIO_BLOCKS_LIBRARIES}
)
