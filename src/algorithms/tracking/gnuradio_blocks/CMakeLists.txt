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

if(ENABLE_CUDA)
     set(OPT_TRACKING_BLOCKS ${OPT_TRACKING_BLOCKS} gps_l1_ca_dll_pll_tracking_gpu_cc.cc)
     set(OPT_TRACKING_INCLUDES ${OPT_TRACKING_INCLUDES} ${CUDA_INCLUDE_DIRS})
     set(OPT_TRACKING_LIBRARIES ${OPT_TRACKING_LIBRARIES} ${CUDA_LIBRARIES})
endif(ENABLE_CUDA)

if(ENABLE_FPGA)
     set(OPT_TRACKING_BLOCKS ${OPT_TRACKING_BLOCKS} gps_l1_ca_dll_pll_c_aid_tracking_fpga_sc.cc)
endif(ENABLE_FPGA)

set(TRACKING_GR_BLOCKS_SOURCES
     galileo_e1_tcp_connector_tracking_cc.cc
     gps_l1_ca_tcp_connector_tracking_cc.cc
     galileo_e5a_dll_pll_tracking_cc.cc
     gps_l2_m_dll_pll_tracking_cc.cc
     gps_l5i_dll_pll_tracking_cc.cc
     gps_l1_ca_dll_pll_c_aid_tracking_cc.cc
     gps_l1_ca_dll_pll_c_aid_tracking_sc.cc
     glonass_l1_ca_dll_pll_tracking_cc.cc
     glonass_l1_ca_dll_pll_c_aid_tracking_cc.cc
     glonass_l1_ca_dll_pll_c_aid_tracking_sc.cc
     gps_l1_ca_kf_tracking_cc.cc
     ${OPT_TRACKING_BLOCKS}   
)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/tracking/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
     ${OPT_TRACKING_INCLUDES}
     ${MATIO_INCLUDE_DIRS}
)

if(ENABLE_GENERIC_ARCH)
    add_definitions( -DGENERIC_ARCH=1 )
endif(ENABLE_GENERIC_ARCH)

file(GLOB TRACKING_GR_BLOCKS_HEADERS "*.h")
list(SORT TRACKING_GR_BLOCKS_HEADERS)
add_library(tracking_gr_blocks ${TRACKING_GR_BLOCKS_SOURCES} ${TRACKING_GR_BLOCKS_HEADERS})
source_group(Headers FILES ${TRACKING_GR_BLOCKS_HEADERS})

target_link_libraries(tracking_gr_blocks tracking_lib ${GNURADIO_RUNTIME_LIBRARIES} gnss_sdr_flags gnss_sp_libs ${Boost_LIBRARIES} ${VOLK_GNSSSDR_LIBRARIES} ${MATIO_LIBRARIES} ${OPT_TRACKING_LIBRARIES})

if(NOT VOLK_GNSSSDR_FOUND)
    if(MATIO_FOUND)
        add_dependencies(tracking_gr_blocks volk_gnsssdr_module)
    else(MATIO_FOUND)
        add_dependencies(tracking_gr_blocks volk_gnsssdr_module matio-${GNSSSDR_MATIO_LOCAL_VERSION})
    endif(MATIO_FOUND)
else(NOT VOLK_GNSSSDR_FOUND)
    if(NOT MATIO_FOUND)
        add_dependencies(tracking_gr_blocks matio-${GNSSSDR_MATIO_LOCAL_VERSION})
    endif(NOT MATIO_FOUND)
endif(NOT VOLK_GNSSSDR_FOUND)
