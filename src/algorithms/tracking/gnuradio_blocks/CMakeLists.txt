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
    set(OPT_TRACKING_BLOCKS_SOURCES
        ${OPT_TRACKING_BLOCKS_SOURCES}
        gps_l1_ca_dll_pll_tracking_gpu_cc.cc
    )
    set(OPT_TRACKING_BLOCKS_HEADERS
        ${OPT_TRACKING_BLOCKS_HEADERS}
        gps_l1_ca_dll_pll_tracking_gpu_cc.h
    )
endif()

if(ENABLE_FPGA)
    set(OPT_TRACKING_BLOCKS_SOURCES
        ${OPT_TRACKING_BLOCKS_SOURCES}
        dll_pll_veml_tracking_fpga.cc
    )
    set(OPT_TRACKING_BLOCKS_HEADERS
        ${OPT_TRACKING_BLOCKS_HEADERS}
        dll_pll_veml_tracking_fpga.h
    )
endif()

set(TRACKING_GR_BLOCKS_SOURCES
    galileo_e1_tcp_connector_tracking_cc.cc
    gps_l1_ca_tcp_connector_tracking_cc.cc
    glonass_l1_ca_dll_pll_tracking_cc.cc
    glonass_l1_ca_dll_pll_c_aid_tracking_cc.cc
    glonass_l1_ca_dll_pll_c_aid_tracking_sc.cc
    gps_l1_ca_kf_tracking_cc.cc
    glonass_l2_ca_dll_pll_tracking_cc.cc
    glonass_l2_ca_dll_pll_c_aid_tracking_cc.cc
    glonass_l2_ca_dll_pll_c_aid_tracking_sc.cc
    dll_pll_veml_tracking.cc
    ${OPT_TRACKING_BLOCKS_SOURCES}
)

set(TRACKING_GR_BLOCKS_HEADERS
    galileo_e1_tcp_connector_tracking_cc.h
    gps_l1_ca_tcp_connector_tracking_cc.h
    glonass_l1_ca_dll_pll_tracking_cc.h
    glonass_l1_ca_dll_pll_c_aid_tracking_cc.h
    glonass_l1_ca_dll_pll_c_aid_tracking_sc.h
    gps_l1_ca_kf_tracking_cc.h
    glonass_l2_ca_dll_pll_tracking_cc.h
    glonass_l2_ca_dll_pll_c_aid_tracking_cc.h
    glonass_l2_ca_dll_pll_c_aid_tracking_sc.h
    dll_pll_veml_tracking.h
    ${OPT_TRACKING_BLOCKS_HEADERS}
)

list(SORT TRACKING_GR_BLOCKS_HEADERS)
list(SORT TRACKING_GR_BLOCKS_SOURCES)

source_group(Headers FILES ${TRACKING_GR_BLOCKS_HEADERS})

add_library(tracking_gr_blocks
    ${TRACKING_GR_BLOCKS_SOURCES}
    ${TRACKING_GR_BLOCKS_HEADERS}
)

if(${FILESYSTEM_FOUND})
    target_compile_definitions(tracking_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM=1)
    if(${find_experimental})
        target_compile_definitions(tracking_gr_blocks PRIVATE -DHAS_STD_FILESYSTEM_EXPERIMENTAL=1)
    endif()
    target_link_libraries(tracking_gr_blocks PRIVATE std::filesystem)
else()
    target_link_libraries(tracking_gr_blocks PRIVATE Boost::filesystem)
endif()

target_link_libraries(tracking_gr_blocks
    PUBLIC
        Boost::boost
        Armadillo::armadillo
        Gnuradio::blocks
        Matio::matio
        Volkgnsssdr::volkgnsssdr
        gnss_sdr_flags
        algorithms_libs
        tracking_libs
)

if(ENABLE_CUDA AND NOT CMAKE_VERSION VERSION_GREATER 3.11)
    target_link_libraries(tracking_gr_blocks
        PUBLIC
            ${CUDA_LIBRARIES}
    )
    target_include_directories(tracking_gr_blocks
        PUBLIC
            ${CUDA_INCLUDE_DIRS}
    )
endif()

if(CMAKE_BUILD_TYPE MATCHES Rel)
    target_compile_definitions(tracking_gr_blocks
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(tracking_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET tracking_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
