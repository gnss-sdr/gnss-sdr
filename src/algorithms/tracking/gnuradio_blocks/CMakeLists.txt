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

#if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#set(TRACKING_GR_BLOCKS_SOURCES 
#     galileo_e1_dll_pll_veml_tracking_cc.cc
#     galileo_e1_tcp_connector_tracking_cc.cc
#     gps_l1_ca_dll_fll_pll_tracking_cc.cc
#     gps_l1_ca_dll_pll_tracking_cc.cc
#     gps_l1_ca_tcp_connector_tracking_cc.cc
#)
#else(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
set(TRACKING_GR_BLOCKS_SOURCES 
     galileo_e1_dll_pll_veml_tracking_cc.cc
     galileo_e1_tcp_connector_tracking_cc.cc
     gps_l1_ca_dll_fll_pll_tracking_cc.cc
     gps_l1_ca_dll_pll_optim_tracking_cc.cc
     gps_l1_ca_dll_pll_tracking_cc.cc
     gps_l1_ca_tcp_connector_tracking_cc.cc
)
#endif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      
include_directories(
     $(CMAKE_CURRENT_SOURCE_DIR)
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${GLOG_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
)

add_library(tracking_gr_blocks ${TRACKING_GR_BLOCKS_SOURCES})
target_link_libraries(tracking_gr_blocks tracking_lib ${GNURADIO_RUNTIME_LIBRARIES} gnss_sp_libs ${Boost_LIBRARIES} )
