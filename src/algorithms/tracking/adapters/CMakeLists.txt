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

if(ENABLE_CUDA)
     set(OPT_TRACKING_ADAPTERS_SOURCES ${OPT_TRACKING_ADAPTERS_SOURCES} gps_l1_ca_dll_pll_tracking_gpu.cc)
     set(OPT_TRACKING_ADAPTERS_HEADERS ${OPT_TRACKING_ADAPTERS_HEADERS} gps_l1_ca_dll_pll_tracking_gpu.h)
     set(OPT_TRACKING_INCLUDE_DIRS ${OPT_TRACKING_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
endif(ENABLE_CUDA)

if(ENABLE_FPGA)
     set(OPT_TRACKING_ADAPTERS_SOURCES ${OPT_TRACKING_ADAPTERS_SOURCES} 
                                       gps_l1_ca_dll_pll_tracking_fpga.cc
                                       gps_l2_m_dll_pll_tracking_fpga.cc
                                       galileo_e1_dll_pll_veml_tracking_fpga.cc
                                       galileo_e5a_dll_pll_tracking_fpga.cc
                                       gps_l5_dll_pll_tracking_fpga.cc)

     set(OPT_TRACKING_ADAPTERS_HEADERS ${OPT_TRACKING_ADAPTERS_HEADERS} 
                                       gps_l1_ca_dll_pll_tracking_fpga.h
                                       gps_l2_m_dll_pll_tracking_fpga.h
                                       galileo_e1_dll_pll_veml_tracking_fpga.h
                                       galileo_e5a_dll_pll_tracking_fpga.h
                                       gps_l5_dll_pll_tracking_fpga.h)                       
endif(ENABLE_FPGA)

set(TRACKING_ADAPTER_SOURCES
     galileo_e1_dll_pll_veml_tracking.cc
     galileo_e1_tcp_connector_tracking.cc
     gps_l1_ca_dll_pll_tracking.cc
     gps_l1_ca_dll_pll_c_aid_tracking.cc
     gps_l1_ca_tcp_connector_tracking.cc
     galileo_e5a_dll_pll_tracking.cc
     gps_l2_m_dll_pll_tracking.cc
     glonass_l1_ca_dll_pll_tracking.cc
     glonass_l1_ca_dll_pll_c_aid_tracking.cc
     gps_l1_ca_kf_tracking.cc
     gps_l5_dll_pll_tracking.cc
     glonass_l2_ca_dll_pll_tracking.cc
     glonass_l2_ca_dll_pll_c_aid_tracking.cc
     ${OPT_TRACKING_ADAPTERS_SOURCES}
)

set(TRACKING_ADAPTER_HEADERS
     galileo_e1_dll_pll_veml_tracking.h
     galileo_e1_tcp_connector_tracking.h
     gps_l1_ca_dll_pll_tracking.h
     gps_l1_ca_dll_pll_c_aid_tracking.h
     gps_l1_ca_tcp_connector_tracking.h
     galileo_e5a_dll_pll_tracking.h
     gps_l2_m_dll_pll_tracking.h
     glonass_l1_ca_dll_pll_tracking.h
     glonass_l1_ca_dll_pll_c_aid_tracking.h
     gps_l1_ca_kf_tracking.h
     gps_l5_dll_pll_tracking.h
     glonass_l2_ca_dll_pll_tracking.h
     glonass_l2_ca_dll_pll_c_aid_tracking.h
     ${OPT_TRACKING_ADAPTERS_HEADERS}
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${ARMADILLO_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${OPT_TRACKING_INCLUDE_DIRS}
)

list(SORT TRACKING_ADAPTER_HEADERS)
list(SORT TRACKING_ADAPTER_SOURCES)

add_library(tracking_adapters ${TRACKING_ADAPTER_SOURCES} ${TRACKING_ADAPTER_HEADERS})
source_group(Headers FILES ${TRACKING_ADAPTER_HEADERS})
target_link_libraries(tracking_adapters tracking_gr_blocks gnss_sp_libs gnss_sdr_flags)
