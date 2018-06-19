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


set(ACQ_ADAPTER_SOURCES
    gps_l1_ca_pcps_acquisition.cc
    gps_l1_ca_pcps_assisted_acquisition.cc
    gps_l1_ca_pcps_acquisition_fine_doppler.cc
    gps_l1_ca_pcps_tong_acquisition.cc
    gps_l1_ca_pcps_quicksync_acquisition.cc
    gps_l2_m_pcps_acquisition.cc
    gps_l5i_pcps_acquisition.cc
    galileo_e1_pcps_ambiguous_acquisition.cc
    galileo_e1_pcps_cccwsr_ambiguous_acquisition.cc
    galileo_e1_pcps_quicksync_ambiguous_acquisition.cc
    galileo_e1_pcps_tong_ambiguous_acquisition.cc
    galileo_e1_pcps_8ms_ambiguous_acquisition.cc
    galileo_e5a_noncoherent_iq_acquisition_caf.cc
    galileo_e5a_pcps_acquisition.cc
    glonass_l1_ca_pcps_acquisition.cc
    glonass_l2_ca_pcps_acquisition.cc
)

if(ENABLE_FPGA)
    set(ACQ_ADAPTER_SOURCES ${ACQ_ADAPTER_SOURCES} gps_l1_ca_pcps_acquisition_fpga.cc)
endif(ENABLE_FPGA)

if(OPENCL_FOUND)
    set(ACQ_ADAPTER_SOURCES ${ACQ_ADAPTER_SOURCES} gps_l1_ca_pcps_opencl_acquisition.cc)
endif(OPENCL_FOUND)

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/gnuradio_blocks
     ${CMAKE_SOURCE_DIR}/src/algorithms/acquisition/libs
     ${CMAKE_SOURCE_DIR}/src/algorithms/libs
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${ARMADILLO_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
     ${GNURADIO_BLOCKS_INCLUDE_DIRS}
     ${VOLK_GNSSSDR_INCLUDE_DIRS}
)

file(GLOB ACQ_ADAPTER_HEADERS "*.h")
list(SORT ACQ_ADAPTER_HEADERS)
add_library(acq_adapters ${ACQ_ADAPTER_SOURCES} ${ACQ_ADAPTER_HEADERS})
source_group(Headers FILES ${ACQ_ADAPTER_HEADERS})
target_link_libraries(acq_adapters acquisition_lib gnss_sp_libs gnss_sdr_flags acq_gr_blocks ${Boost_LIBRARIES} ${GNURADIO_RUNTIME_LIBRARIES} ${GNURADIO_BLOCKS_LIBRARIES})
