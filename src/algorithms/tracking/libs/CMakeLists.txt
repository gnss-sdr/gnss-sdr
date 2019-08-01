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
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        set(TRACKING_LIB_SOURCES ${TRACKING_LIB_SOURCES} cuda_multicorrelator.cu)
        set(TRACKING_LIB_HEADERS ${TRACKING_LIB_HEADERS} cuda_multicorrelator.h)
    else()
        list(APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30; -std=c++11;-O3; -use_fast_math -default-stream per-thread")
        set(CUDA_PROPAGATE_HOST_FLAGS OFF)
        cuda_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
        set(LIB_TYPE STATIC) #set the lib type
        cuda_add_library(CUDA_CORRELATOR_LIB ${LIB_TYPE} cuda_multicorrelator.h cuda_multicorrelator.cu)
        set(OPT_TRACKING_LIBRARIES ${OPT_TRACKING_LIBRARIES} CUDA_CORRELATOR_LIB)
        set(OPT_TRACKING_INCLUDES ${OPT_TRACKING_INCLUDES} ${CUDA_INCLUDE_DIRS})
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

source_group(Headers FILES ${TRACKING_LIB_HEADERS})

add_library(tracking_libs ${TRACKING_LIB_SOURCES} ${TRACKING_LIB_HEADERS})

target_link_libraries(tracking_libs
    PUBLIC
        Armadillo::armadillo
        Boost::boost
        Gnuradio::runtime
        Volkgnsssdr::volkgnsssdr
        core_system_parameters
        algorithms_libs
        ${OPT_TRACKING_LIBRARIES}
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(NOT CMAKE_VERSION VERSION_GREATER 3.11)
    target_link_libraries(tracking_libs
        PUBLIC
            ${OPT_TRACKING_LIBRARIES}
    )
    target_include_directories(tracking_libs
        PUBLIC
            ${OPT_TRACKING_INCLUDES}
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.65.99)
    target_compile_definitions(tracking_libs
        PUBLIC
            -DBOOST_GREATER_1_65
    )
endif()

# Fix for Boost Asio < 1.70
if(OS_IS_MACOSX)
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

if(CMAKE_BUILD_TYPE MATCHES Rel)
    target_compile_definitions(tracking_libs
        PUBLIC -DARMA_NO_BOUND_CHECKING=1
    )
endif()

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
