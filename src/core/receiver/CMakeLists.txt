# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


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
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/acquisition_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/channel_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/configuration_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/gnss_block_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/observables_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/pvt_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/telemetry_decoder_interface.h
    ${GNSSSDR_SOURCE_DIR}/src/core/interfaces/tracking_interface.h
)

list(SORT GNSS_RECEIVER_INTERFACE_HEADERS)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_receiver STATIC)
    target_sources(core_receiver
        PRIVATE
            ${GNSS_RECEIVER_SOURCES}
        PUBLIC
            ${GNSS_RECEIVER_HEADERS}
        INTERFACE
            ${GNSS_RECEIVER_INTERFACE_HEADERS}
    )
else()
    source_group(Headers FILES ${GNSS_RECEIVER_HEADERS}
        ${GNSS_RECEIVER_INTERFACE_HEADERS}
    )
    add_library(core_receiver
        ${GNSS_RECEIVER_SOURCES}
        ${GNSS_RECEIVER_HEADERS}
        ${GNSS_RECEIVER_INTERFACE_HEADERS}
    )
endif()

if(CMAKE_ANDROID_ARCH_ABI)
        target_compile_definitions(core_receiver PUBLIC -DANDROID=1)
endif()

if(ENABLE_FPGA)
    target_compile_definitions(core_receiver PUBLIC -DENABLE_FPGA=1)
endif()

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(core_receiver PUBLIC -DGNURADIO_USES_STD_POINTERS=1)
endif()

if(ENABLE_RAW_UDP)
    target_compile_definitions(core_receiver PRIVATE -DRAW_UDP=1)
endif()

if(GNURADIO_IS_38_OR_GREATER)
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

if(ENABLE_MAX2771)
    target_compile_definitions(core_receiver PRIVATE -DMAX2771_DRIVER=1)
endif()

if(ENABLE_DMA_PROXY)
    target_compile_definitions(core_receiver PRIVATE -DDMA_PROXY_DRIVER=1)
endif()

if(ENABLE_OSMOSDR)
    if(GROSMOSDR_FOUND)
        target_compile_definitions(core_receiver PRIVATE -DOSMOSDR_DRIVER=1)
    endif()
endif()

if(ENABLE_LIMESDR)
    target_compile_definitions(core_receiver PRIVATE -DLIMESDR_DRIVER=1)
endif()

if(ENABLE_ARRAY)
    target_compile_definitions(core_receiver PRIVATE -DRAW_ARRAY_DRIVER=1)
endif()

if(ENABLE_ZMQ)
    target_compile_definitions(core_receiver PRIVATE -DZEROMQ_DRIVER=1)
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
    if(CMAKE_VERSION VERSION_GREATER 3.11)
        target_include_directories(core_receiver
            PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        )
    else()
        target_link_libraries(core_receiver
            PUBLIC ${CUDA_LIBRARIES}
        )
        target_include_directories(core_receiver
            PUBLIC ${CUDA_INCLUDE_DIRS}
        )
    endif()
    set_target_properties(core_receiver PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        POSITION_INDEPENDENT_CODE ON
    )
    target_compile_definitions(core_receiver PRIVATE -DCUDA_GPU_ACCEL=1)
endif()

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(core_receiver
        PRIVATE
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

target_link_libraries(core_receiver
    PUBLIC
        core_libs
    PRIVATE
        algorithms_libs
        core_monitor
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
        gnss_sdr_flags
        Boost::headers
        Armadillo::armadillo
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(core_receiver PRIVATE Gflags::gflags Glog::glog)
    target_compile_definitions(core_receiver PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(core_receiver PRIVATE absl::flags absl::log)
endif()

if(ENABLE_ARMA_NO_DEBUG)
    target_compile_definitions(core_receiver
        PRIVATE -DARMA_NO_BOUND_CHECKING=1
    )
endif()

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(core_receiver
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(core_receiver
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(PMT_USES_BOOST_ANY)
    target_compile_definitions(core_receiver
        PRIVATE
            -DPMT_USES_BOOST_ANY=1
    )
endif()

# Fix for Boost Asio < 1.70
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
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
    $<BUILD_INTERFACE:${GNSSSDR_SOURCE_DIR}/src/core/receiver>
    $<BUILD_INTERFACE:${GNSSSDR_SOURCE_DIR}/src/core/interfaces>
)
