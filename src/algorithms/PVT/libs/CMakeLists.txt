# Copyright (C) 2012-2013  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# GNSS-SDR is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option) any later version.
#
# GNSS-SDR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
#

set(PVT_LIB_SOURCES 
     gps_l1_ca_ls_pvt.cc
     galileo_e1_ls_pvt.cc
     kml_printer.cc
     rinex_printer.cc
     nmea_printer.cc  
#     rtcm_printer.cc  
)

include_directories(
     $(CMAKE_CURRENT_SOURCE_DIR)
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/PVT/adapters
     ${Boost_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
)

add_library(pvt_lib ${PVT_LIB_SOURCES})
target_link_libraries(pvt_lib ${Boost_LIBRARIES} ${GFlags_LIBS} ${GLOG_LIBRARIES} ${ARMADILLO_LIBRARIES} )
add_dependencies(input_filter_adapters armadillo-${armadillo_RELEASE} glog-${GLOG_RELEASE})