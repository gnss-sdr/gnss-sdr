# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(OPT_SIGNAL_SOURCE_LIB_SOURCES "")
set(OPT_SIGNAL_SOURCE_LIB_HEADERS "")
if(ENABLE_FMCOMMS2 OR ENABLE_AD9361)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad9361_manager.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad9361_manager.h)
endif()

if(ENABLE_MAX2771)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_spidev.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_spidev.h)
endif()

if(ENABLE_FPGA)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_switch.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_HEADERS} fpga_switch.h)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_dynamic_bit_selection.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_HEADERS} fpga_dynamic_bit_selection.h)
endif()

if(ENABLE_DMA_PROXY)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_dma-proxy.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_HEADERS} fpga_dma-proxy.h)
endif()

if((ENABLE_FPGA AND ENABLE_AD9361) OR ENABLE_MAX2771)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} fpga_buffer_monitor.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_HEADERS} fpga_buffer_monitor.h)
endif()

if(ENABLE_PLUTOSDR)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad936x_iio_samples.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad936x_iio_samples.h)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad936x_iio_custom.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ad936x_iio_custom.h)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} pps_samplestamp.h)
    set(OPT_SIGNAL_SOURCE_LIB_SOURCES ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ppstcprx.cc)
    set(OPT_SIGNAL_SOURCE_LIB_HEADERS ${OPT_SIGNAL_SOURCE_LIB_SOURCES} ppstcprx.h)
endif()


set(SIGNAL_SOURCE_LIB_SOURCES
    rtl_tcp_commands.cc
    rtl_tcp_dongle_info.cc
    gnss_sdr_valve.cc
    gnss_sdr_timestamp.cc
    ${OPT_SIGNAL_SOURCE_LIB_SOURCES}
)

set(SIGNAL_SOURCE_LIB_HEADERS
    rtl_tcp_commands.h
    rtl_tcp_dongle_info.h
    gnss_sdr_valve.h
    ${OPT_SIGNAL_SOURCE_LIB_HEADERS}
)

list(SORT SIGNAL_SOURCE_LIB_HEADERS)
list(SORT SIGNAL_SOURCE_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(signal_source_libs STATIC)
    target_sources(signal_source_libs
        PRIVATE
            ${SIGNAL_SOURCE_LIB_SOURCES}
        PUBLIC
            ${SIGNAL_SOURCE_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${SIGNAL_SOURCE_LIB_HEADERS})
    add_library(signal_source_libs
        ${SIGNAL_SOURCE_LIB_SOURCES}
        ${SIGNAL_SOURCE_LIB_HEADERS}
    )
endif()

target_link_libraries(signal_source_libs
    PUBLIC
        Boost::headers
        Gnuradio::runtime
    PRIVATE
        core_libs
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(signal_source_libs PRIVATE Gflags::gflags Glog::glog)
    target_compile_definitions(signal_source_libs PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(signal_source_libs PRIVATE absl::flags absl::log)
endif()

target_include_directories(signal_source_libs
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/receiver
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
        ${GNSSSDR_SOURCE_DIR}/src/algorithms/libs
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_source_libs
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(ENABLE_FMCOMMS2 OR ENABLE_AD9361 OR ENABLE_PLUTOSDR)
    target_link_libraries(signal_source_libs
        PUBLIC
            Iio::iio
        PRIVATE
            Iio::ad9361
        )
    if(LIBAD9361_VERSION)
        if(LIBAD9361_VERSION VERSION_GREATER 0.1)
            target_compile_definitions(signal_source_libs
                PRIVATE -DLIBAD9361_VERSION_GREATER_THAN_01=1
            )
        endif()
    endif()
endif()

if(ENABLE_FPGA OR ENABLE_AD9361)
    target_link_libraries(signal_source_libs
        PUBLIC
            algorithms_libs
        )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_source_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_source_libs APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
