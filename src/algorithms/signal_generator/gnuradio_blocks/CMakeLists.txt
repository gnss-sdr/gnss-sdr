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

set(SIGNAL_GENERATOR_BLOCK_SOURCES signal_generator_c.cc)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

file(GLOB SIGNAL_GENERATOR_BLOCK_HEADERS "*.h")
list(SORT SIGNAL_GENERATOR_BLOCK_HEADERS)
add_library(signal_generator_blocks ${SIGNAL_GENERATOR_BLOCK_SOURCES} ${SIGNAL_GENERATOR_BLOCK_HEADERS})
source_group(Headers FILES ${SIGNAL_GENERATOR_BLOCK_HEADERS})
target_link_libraries(signal_generator_blocks gnss_system_parameters gnss_sp_libs
                                              ${GNURADIO_RUNTIME_LIBRARIES}
                                              ${GNURADIO_FFT_LIBRARIES}
                                              ${VOLK_GNSSSDR_LIBRARIES} ${ORC_LIBRARIES}
)

if(VOLK_GNSSSDR_FOUND)
    # add_dependencies(signal_generator_blocks glog-${glog_RELEASE})
else(VOLK_GNSSSDR_FOUND)
    add_dependencies(signal_generator_blocks volk_gnsssdr_module)
endif()