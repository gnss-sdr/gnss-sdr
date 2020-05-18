# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(OPT_TRACKING_BLOCKS_SOURCES "")
set(OPT_TRACKING_BLOCKS_HEADERS "")

if(ENABLE_CUDA)
    set(OPT_TRACKING_BLOCKS_SOURCES
        gps_l1_ca_dll_pll_tracking_gpu_cc.cc
    )
    set(OPT_TRACKING_BLOCKS_HEADERS
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
        Boost::headers
        Armadillo::armadillo
        Gnuradio::blocks
        Volkgnsssdr::volkgnsssdr
        algorithms_libs
        tracking_libs
    PRIVATE
        Matio::matio
        gnss_sdr_flags
        Glog::glog
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(tracking_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

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

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(tracking_gr_blocks
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

if(CMAKE_VERSION VERSION_GREATER 3.1)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(tracking_gr_blocks
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(tracking_gr_blocks
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.72.99)
    target_compile_definitions(tracking_gr_blocks
        PRIVATE
            -DBOOST_173_OR_GREATER=1
    )
endif()

if(has_span)
    target_compile_definitions(tracking_gr_blocks
        PUBLIC -DHAS_STD_SPAN=1
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
