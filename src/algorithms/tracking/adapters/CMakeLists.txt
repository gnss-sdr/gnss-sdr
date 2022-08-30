# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

set(OPT_TRACKING_ADAPTERS_SOURCES "")
set(OPT_TRACKING_ADAPTERS_HEADERS "")
if(ENABLE_CUDA)
    set(OPT_TRACKING_ADAPTERS_SOURCES
        gps_l1_ca_dll_pll_tracking_gpu.cc
    )
    set(OPT_TRACKING_ADAPTERS_HEADERS
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
    galileo_e5b_dll_pll_tracking.cc
    galileo_e6_dll_pll_tracking.cc
    gps_l2_m_dll_pll_tracking.cc
    glonass_l1_ca_dll_pll_tracking.cc
    glonass_l1_ca_dll_pll_c_aid_tracking.cc
    gps_l1_ca_gaussian_tracking.cc
    gps_l5_dll_pll_tracking.cc
    glonass_l2_ca_dll_pll_tracking.cc
    glonass_l2_ca_dll_pll_c_aid_tracking.cc
    beidou_b1i_dll_pll_tracking.cc
    beidou_b3i_dll_pll_tracking.cc
    gps_l1_ca_kf_tracking.cc
    ${OPT_TRACKING_ADAPTERS_SOURCES}
)

set(TRACKING_ADAPTER_HEADERS
    galileo_e1_dll_pll_veml_tracking.h
    galileo_e1_tcp_connector_tracking.h
    gps_l1_ca_dll_pll_tracking.h
    gps_l1_ca_tcp_connector_tracking.h
    galileo_e5a_dll_pll_tracking.h
    galileo_e5b_dll_pll_tracking.h
    galileo_e6_dll_pll_tracking.h
    gps_l2_m_dll_pll_tracking.h
    glonass_l1_ca_dll_pll_tracking.h
    glonass_l1_ca_dll_pll_c_aid_tracking.h
    gps_l1_ca_gaussian_tracking.h
    gps_l5_dll_pll_tracking.h
    glonass_l2_ca_dll_pll_tracking.h
    glonass_l2_ca_dll_pll_c_aid_tracking.h
    beidou_b1i_dll_pll_tracking.h
    beidou_b3i_dll_pll_tracking.h
    gps_l1_ca_kf_tracking.h
    ${OPT_TRACKING_ADAPTERS_HEADERS}
)

list(SORT TRACKING_ADAPTER_HEADERS)
list(SORT TRACKING_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(tracking_adapters STATIC)
    target_sources(tracking_adapters
        PRIVATE
            ${TRACKING_ADAPTER_SOURCES}
        PUBLIC
            ${TRACKING_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${TRACKING_ADAPTER_HEADERS})
    add_library(tracking_adapters
        ${TRACKING_ADAPTER_SOURCES}
        ${TRACKING_ADAPTER_HEADERS}
    )
endif()

target_link_libraries(tracking_adapters
    PUBLIC
        tracking_gr_blocks
    PRIVATE
        gnss_sdr_flags
        Glog::glog
)

target_include_directories(tracking_adapters
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_FPGA)
    target_link_libraries(tracking_adapters
        PRIVATE
            core_libs
    )
endif()

if(ENABLE_CUDA)
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        target_include_directories(tracking_adapters
            PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        )
    else()
        target_link_libraries(tracking_adapters
            PUBLIC ${CUDA_LIBRARIES}
        )
        target_include_directories(tracking_adapters
            PUBLIC ${CUDA_INCLUDE_DIRS}
        )
    endif()
    set_target_properties(tracking_adapters PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        POSITION_INDEPENDENT_CODE ON
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
