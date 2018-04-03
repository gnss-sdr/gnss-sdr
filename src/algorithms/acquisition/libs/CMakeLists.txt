# Copyright (C) 2012-2015  (see AUTHORS file for a list of contributors)
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
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

  
set(ACQUISITION_LIB_SOURCES   
	gps_fpga_acquisition_8sc.cc
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${VOLK_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

file(GLOB ACQUISITION_LIB_HEADERS "*.h")
list(SORT ACQUISITION_LIB_HEADERS)
add_library(acquisition_lib ${ACQUISITION_LIB_SOURCES} ${ACQUISITION_LIB_HEADERS})
source_group(Headers FILES ${ACQUISITION_LIB_HEADERS})
target_link_libraries(acquisition_lib ${VOLK_LIBRARIES} ${VOLK_GNSSSDR_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES})
if(VOLK_GNSSSDR_FOUND)
    add_dependencies(acquisition_lib glog-${glog_RELEASE})
else(VOLK_GNSSSDR_FOUND)
    add_dependencies(acquisition_lib glog-${glog_RELEASE} volk_gnsssdr_module)
endif()

