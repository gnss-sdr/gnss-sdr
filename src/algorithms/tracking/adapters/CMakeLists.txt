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


if(ENABLE_CUDA)
    set(OPT_TRACKING_ADAPTERS_SOURCES
        ${OPT_TRACKING_ADAPTERS_SOURCES}
        gps_l1_ca_dll_pll_tracking_gpu.cc
    )
    set(OPT_TRACKING_ADAPTERS_HEADERS
        ${OPT_TRACKING_ADAPTERS_HEADERS}
        gps_l1_ca_dll_pll_tracking_gpu.h
    )
endif()

if(ENABLE_FPGA)
    set(OPT_TRACKING_ADAPTERS_SOURCES
        ${OPT_TRACKING_ADAPTERS_SOURCES}
        gps_l1_ca_dll_pll_tracking_fpga.cc
        gps_l2_m_dll_pll_tracking_fpga.cc
        galileo_e1_dll_pll_veml_tracking_fpga.cc
        galileo_e5a_dll_pll_tracking_fpga.cc
        gps_l5_dll_pll_tracking_fpga.cc
    )
    set(OPT_TRACKING_ADAPTERS_HEADERS
        ${OPT_TRACKING_ADAPTERS_HEADERS}
        gps_l1_ca_dll_pll_tracking_fpga.h
        gps_l2_m_dll_pll_tracking_fpga.h
        galileo_e1_dll_pll_veml_tracking_fpga.h
        galileo_e5a_dll_pll_tracking_fpga.h
        gps_l5_dll_pll_tracking_fpga.h
    )
endif()

set(TRACKING_ADAPTER_SOURCES
    galileo_e1_dll_pll_veml_tracking.cc
    galileo_e1_tcp_connector_tracking.cc
    gps_l1_ca_dll_pll_tracking.cc
    gps_l1_ca_tcp_connector_tracking.cc
    galileo_e5a_dll_pll_tracking.cc
    gps_l2_m_dll_pll_tracking.cc
    glonass_l1_ca_dll_pll_tracking.cc
    glonass_l1_ca_dll_pll_c_aid_tracking.cc
    gps_l1_ca_kf_tracking.cc
    gps_l5_dll_pll_tracking.cc
    glonass_l2_ca_dll_pll_tracking.cc
    glonass_l2_ca_dll_pll_c_aid_tracking.cc
    beidou_b1i_dll_pll_tracking.cc
    beidou_b3i_dll_pll_tracking.cc
    ${OPT_TRACKING_ADAPTERS_SOURCES}
)

set(TRACKING_ADAPTER_HEADERS
    galileo_e1_dll_pll_veml_tracking.h
    galileo_e1_tcp_connector_tracking.h
    gps_l1_ca_dll_pll_tracking.h
    gps_l1_ca_tcp_connector_tracking.h
    galileo_e5a_dll_pll_tracking.h
    gps_l2_m_dll_pll_tracking.h
    glonass_l1_ca_dll_pll_tracking.h
    glonass_l1_ca_dll_pll_c_aid_tracking.h
    gps_l1_ca_kf_tracking.h
    gps_l5_dll_pll_tracking.h
    glonass_l2_ca_dll_pll_tracking.h
    glonass_l2_ca_dll_pll_c_aid_tracking.h
    beidou_b1i_dll_pll_tracking.h
    beidou_b3i_dll_pll_tracking.h
    ${OPT_TRACKING_ADAPTERS_HEADERS}
)

list(SORT TRACKING_ADAPTER_HEADERS)
list(SORT TRACKING_ADAPTER_SOURCES)

source_group(Headers FILES ${TRACKING_ADAPTER_HEADERS})

add_library(tracking_adapters
    ${TRACKING_ADAPTER_SOURCES}
    ${TRACKING_ADAPTER_HEADERS}
)

target_link_libraries(tracking_adapters
    PUBLIC
        tracking_gr_blocks
        algorithms_libs
        gnss_sdr_flags
        Gnuradio::runtime
    PRIVATE
        Volkgnsssdr::volkgnsssdr
)

target_include_directories(tracking_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CUDA AND NOT CMAKE_VERSION VERSION_GREATER 3.11)
    target_link_libraries(tracking_adapters
        PUBLIC
            ${CUDA_LIBRARIES}
    )
    target_include_directories(tracking_adapters
        PUBLIC
            ${CUDA_INCLUDE_DIRS}
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(tracking_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET tracking_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
