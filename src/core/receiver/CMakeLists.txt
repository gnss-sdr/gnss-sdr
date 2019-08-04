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


set(GNSS_RECEIVER_SOURCES
    control_thread.cc
    file_configuration.cc
    gnss_block_factory.cc
    gnss_flowgraph.cc
    in_memory_configuration.cc
    tcp_cmd_interface.cc
)

set(GNSS_RECEIVER_HEADERS
    control_thread.h
    file_configuration.h
    gnss_block_factory.h
    gnss_flowgraph.h
    in_memory_configuration.h
    tcp_cmd_interface.h
    concurrent_map.h
    concurrent_queue.h
)

list(SORT GNSS_RECEIVER_HEADERS)
list(SORT GNSS_RECEIVER_SOURCES)

set(GNSS_RECEIVER_INTERFACE_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/acquisition_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/channel_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/configuration_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/gnss_block_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/observables_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/pvt_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/telemetry_decoder_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../interfaces/tracking_interface.h
)

list(SORT GNSS_RECEIVER_INTERFACE_HEADERS)

source_group(Headers FILES ${GNSS_RECEIVER_HEADERS}
    ${GNSS_RECEIVER_INTERFACE_HEADERS}
)

add_library(core_receiver
    ${GNSS_RECEIVER_SOURCES}
    ${GNSS_RECEIVER_HEADERS}
    ${GNSS_RECEIVER_INTERFACE_HEADERS}
)

if(ENABLE_FPGA)
    target_compile_definitions(core_receiver PUBLIC -DENABLE_FPGA=1)
endif()

if(ENABLE_RAW_UDP)
    target_compile_definitions(core_receiver PRIVATE -DRAW_UDP=1)
endif()

if(NOT (GNURADIO_VERSION VERSION_LESS "3.8"))
    target_compile_definitions(core_receiver PRIVATE -DGR_GREATER_38=1)
endif()

if(ENABLE_UHD AND GNURADIO_UHD_LIBRARIES_gnuradio-uhd)
    target_compile_definitions(core_receiver PRIVATE -DUHD_DRIVER=1)
endif()

if(ENABLE_PLUTOSDR)
    target_compile_definitions(core_receiver PRIVATE -DPLUTOSDR_DRIVER=1)
endif()

if(ENABLE_FMCOMMS2)
    target_compile_definitions(core_receiver PRIVATE -DFMCOMMS2_DRIVER=1)
endif()

if(ENABLE_AD9361)
    target_compile_definitions(core_receiver PRIVATE -DAD9361_DRIVER=1)
endif()

if(ENABLE_OSMOSDR)
    if(GROSMOSDR_FOUND)
        target_compile_definitions(core_receiver PRIVATE -DOSMOSDR_DRIVER=1)
    endif()
endif()

if(ENABLE_GN3S)
    target_compile_definitions(core_receiver PRIVATE -DGN3S_DRIVER=1)
endif()

if(ENABLE_ARRAY)
    target_compile_definitions(core_receiver PRIVATE -DRAW_ARRAY_DRIVER=1)
endif()

if(ENABLE_FLEXIBAND)
    target_compile_definitions(core_receiver PRIVATE -DFLEXIBAND_DRIVER=1)
endif()

if(ENABLE_OPENCL)
    message(STATUS "Adding processing blocks implemented using OpenCL")
    target_link_libraries(core_receiver PUBLIC algorithms_libs OpenCL::OpenCL)
    target_compile_definitions(core_receiver PRIVATE -DOPENCL_BLOCKS=1)
else()
    target_compile_definitions(core_receiver PRIVATE -DOPENCL_BLOCKS=0)
endif()

if(ENABLE_CUDA)
    if(NOT CMAKE_VERSION VERSION_GREATER 3.11)
        target_include_directories(core_receiver PUBLIC ${CUDA_INCLUDE_DIRS})
    endif()
    target_compile_definitions(core_receiver PRIVATE -DCUDA_GPU_ACCEL=1)
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.65.99)
    target_compile_definitions(core_receiver
        PRIVATE
            -DBOOST_GREATER_1_65
    )
endif()

target_link_libraries(core_receiver
    PUBLIC
        core_libs
        core_monitor
        Gnuradio::runtime
    PRIVATE
        signal_source_adapters
        data_type_adapters
        input_filter_adapters
        conditioner_adapters
        resampler_adapters
        acquisition_adapters
        tracking_adapters
        channel_adapters
        telemetry_decoder_adapters
        obs_adapters
        pvt_adapters
        Boost::boost
        Gflags::gflags
        Glog::glog
        Armadillo::armadillo
)

if(CMAKE_BUILD_TYPE MATCHES Rel)
    target_compile_definitions(core_receiver
        PRIVATE -DARMA_NO_BOUND_CHECKING=1
    )
endif()

# Fix for Boost Asio < 1.70
if(OS_IS_MACOSX)
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(core_receiver
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(core_receiver
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

if(ENABLE_FPGA)
    target_link_libraries(core_receiver
        PUBLIC
            Boost::thread
        PRIVATE
            Boost::chrono
    )
endif()

if(ENABLE_FLEXIBAND AND TELEORBIT_FOUND)
    target_link_libraries(core_receiver
        PRIVATE
            Gnuradio::teleorbit
    )
endif()

if(ENABLE_GN3S AND GRGN3S_FOUND)
    target_link_libraries(core_receiver
        PRIVATE
            Gnuradio::gn3s
    )
endif()

if(ENABLE_ARRAY AND GRDBFCTTC_FOUND)
    target_link_libraries(core_receiver
        PRIVATE
            Gnuradio::dbfcttc
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(core_receiver
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET core_receiver APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core/receiver>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core/interfaces>
)
