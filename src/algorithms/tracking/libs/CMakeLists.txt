# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(TRACKING_LIB_SOURCES
    cpu_multicorrelator.cc
    cpu_multicorrelator_real_codes.cc
    cpu_multicorrelator_16sc.cc
    lock_detectors.cc
    tcp_communication.cc
    tcp_packet_data.cc
    tracking_2nd_DLL_filter.cc
    tracking_2nd_PLL_filter.cc
    tracking_discriminators.cc
    tracking_FLL_PLL_filter.cc
    tracking_loop_filter.cc
    dll_pll_conf.cc
    bayesian_estimation.cc
    exponential_smoother.cc
)

set(TRACKING_LIB_HEADERS
    cpu_multicorrelator.h
    cpu_multicorrelator_real_codes.h
    cpu_multicorrelator_16sc.h
    lock_detectors.h
    tcp_communication.h
    tcp_packet_data.h
    tracking_2nd_DLL_filter.h
    tracking_2nd_PLL_filter.h
    tracking_discriminators.h
    tracking_FLL_PLL_filter.h
    tracking_loop_filter.h
    dll_pll_conf.h
    bayesian_estimation.h
    exponential_smoother.h
)

if(ENABLE_CUDA)
    list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30; -O3; -use_fast_math -default-stream per-thread")
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        set(TRACKING_LIB_SOURCES ${TRACKING_LIB_SOURCES} cuda_multicorrelator.cu)
        set(TRACKING_LIB_HEADERS ${TRACKING_LIB_HEADERS} cuda_multicorrelator.h)
    else()
        cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        cuda_add_library(cuda_correlator_lib STATIC cuda_multicorrelator.h cuda_multicorrelator.cu)
    endif()
endif()

if(ARMADILLO_VERSION_STRING VERSION_GREATER 7.400)
    # sqrtmat_sympd() requires 7.400
    set(TRACKING_LIB_SOURCES ${TRACKING_LIB_SOURCES} nonlinear_tracking.cc)
    set(TRACKING_LIB_HEADERS ${TRACKING_LIB_HEADERS} nonlinear_tracking.h)
endif()

if(ENABLE_FPGA)
    set(TRACKING_LIB_SOURCES ${TRACKING_LIB_SOURCES} fpga_multicorrelator.cc dll_pll_conf_fpga.cc)
    set(TRACKING_LIB_HEADERS ${TRACKING_LIB_HEADERS} fpga_multicorrelator.h dll_pll_conf_fpga.h)
endif()

list(SORT TRACKING_LIB_HEADERS)
list(SORT TRACKING_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(tracking_libs STATIC)
    target_sources(tracking_libs
        PRIVATE
            ${TRACKING_LIB_SOURCES}
        PUBLIC
            ${TRACKING_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${TRACKING_LIB_HEADERS})
    add_library(tracking_libs ${TRACKING_LIB_SOURCES} ${TRACKING_LIB_HEADERS})
endif()

target_link_libraries(tracking_libs
    PUBLIC
        Armadillo::armadillo
        Boost::headers
        Gnuradio::runtime
        Volkgnsssdr::volkgnsssdr
        core_system_parameters
        algorithms_libs
    PRIVATE
        gnss_sdr_flags
        Glog::glog
)

if(ENABLE_CUDA)
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        target_include_directories(tracking_libs
            PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        )
    else()
        target_link_libraries(tracking_libs
            PUBLIC ${CUDA_LIBRARIES}
        )
        target_include_directories(tracking_libs
            PUBLIC ${CUDA_INCLUDE_DIRS}
        )
    endif()
    set_target_properties(tracking_libs PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        POSITION_INDEPENDENT_CODE ON
        CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )
endif()

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(tracking_libs
        PUBLIC
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

# Fix for Boost Asio < 1.70
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(tracking_libs
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(tracking_libs
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(tracking_libs
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

target_include_directories(tracking_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(tracking_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET tracking_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
