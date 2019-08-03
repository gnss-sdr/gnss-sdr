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
    beidou_b1i_pcps_acquisition.cc
    beidou_b3i_pcps_acquisition.cc
)

set(ACQ_ADAPTER_HEADERS
    gps_l1_ca_pcps_acquisition.h
    gps_l1_ca_pcps_assisted_acquisition.h
    gps_l1_ca_pcps_acquisition_fine_doppler.h
    gps_l1_ca_pcps_tong_acquisition.h
    gps_l1_ca_pcps_quicksync_acquisition.h
    gps_l2_m_pcps_acquisition.h
    gps_l5i_pcps_acquisition.h
    galileo_e1_pcps_ambiguous_acquisition.h
    galileo_e1_pcps_cccwsr_ambiguous_acquisition.h
    galileo_e1_pcps_quicksync_ambiguous_acquisition.h
    galileo_e1_pcps_tong_ambiguous_acquisition.h
    galileo_e1_pcps_8ms_ambiguous_acquisition.h
    galileo_e5a_noncoherent_iq_acquisition_caf.h
    galileo_e5a_pcps_acquisition.h
    glonass_l1_ca_pcps_acquisition.h
    glonass_l2_ca_pcps_acquisition.h
    beidou_b1i_pcps_acquisition.h
    beidou_b3i_pcps_acquisition.h
)

if(ENABLE_FPGA)
    set(ACQ_ADAPTER_SOURCES ${ACQ_ADAPTER_SOURCES}
        gps_l1_ca_pcps_acquisition_fpga.cc
        gps_l2_m_pcps_acquisition_fpga.cc
        galileo_e1_pcps_ambiguous_acquisition_fpga.cc
        galileo_e5a_pcps_acquisition_fpga.cc
        gps_l5i_pcps_acquisition_fpga.cc
    )
    set(ACQ_ADAPTER_HEADERS ${ACQ_ADAPTER_HEADERS}
        gps_l1_ca_pcps_acquisition_fpga.h
        gps_l2_m_pcps_acquisition_fpga.h
        galileo_e1_pcps_ambiguous_acquisition_fpga.h
        galileo_e5a_pcps_acquisition_fpga.h
        gps_l5i_pcps_acquisition_fpga.h
    )
endif()

if(ENABLE_OPENCL)
    set(ACQ_ADAPTER_SOURCES
        ${ACQ_ADAPTER_SOURCES}
        gps_l1_ca_pcps_opencl_acquisition.cc
    )
    set(ACQ_ADAPTER_HEADERS ${ACQ_ADAPTER_HEADERS}
        gps_l1_ca_pcps_opencl_acquisition.h
    )
endif()

list(SORT ACQ_ADAPTER_HEADERS)
list(SORT ACQ_ADAPTER_SOURCES)

source_group(Headers FILES ${ACQ_ADAPTER_HEADERS})

add_library(acquisition_adapters ${ACQ_ADAPTER_SOURCES} ${ACQ_ADAPTER_HEADERS})

target_link_libraries(acquisition_adapters
    PUBLIC
        algorithms_libs
        gnss_sdr_flags
        acquisition_gr_blocks
        acquisition_libs
        channel_libs
        core_system_parameters
        Gnuradio::blocks
        Volk::volk
    PRIVATE
        Boost::boost
        Gflags::gflags
        Glog::glog
        Gnuradio::fft
        Volkgnsssdr::volkgnsssdr
)

target_include_directories(acquisition_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(acquisition_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET acquisition_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
