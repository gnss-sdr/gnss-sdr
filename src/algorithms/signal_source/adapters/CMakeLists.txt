# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


# Optional drivers

if(ENABLE_RAW_UDP AND PCAP_FOUND)
    list(APPEND OPT_DRIVER_SOURCES custom_udp_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS custom_udp_signal_source.h)
endif()

if(ENABLE_PLUTOSDR)
    ##############################################
    # ADALM-PLUTO (Analog Devices Inc.)
    ##############################################
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} plutosdr_signal_source.cc)
    set(OPT_DRIVER_HEADERS ${OPT_DRIVER_HEADERS} plutosdr_signal_source.h)
    ##############################################
    # CUSTOM AD936X IIO SOURCE
    ##############################################
    set(OPT_DRIVER_SOURCES ${OPT_DRIVER_SOURCES} ad936x_custom_signal_source.cc)
    set(OPT_DRIVER_HEADERS ${OPT_DRIVER_HEADERS} ad936x_custom_signal_source.h)
endif()

if(ENABLE_FMCOMMS2)
    ###############################################
    # FMCOMMS2 based SDR Hardware
    ###############################################
    list(APPEND OPT_DRIVER_SOURCES fmcomms2_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS fmcomms2_signal_source.h)
endif()

if(ENABLE_AD9361)
    ###############################################
    # AD9361 DIRECT TO FPGA Hardware
    ###############################################
    list(APPEND OPT_DRIVER_SOURCES adrv9361_z7035_signal_source_fpga.cc)
    list(APPEND OPT_DRIVER_HEADERS adrv9361_z7035_signal_source_fpga.h)
    list(APPEND OPT_DRIVER_SOURCES fmcomms5_signal_source_fpga.cc)
    list(APPEND OPT_DRIVER_HEADERS fmcomms5_signal_source_fpga.h)
endif()

if(ENABLE_MAX2771)
    ###############################################
    # MAX2771 EVKIT DIRECT TO FPGA Hardware
    ###############################################
    list(APPEND OPT_DRIVER_SOURCES max2771_evkit_signal_source_fpga.cc)
    list(APPEND OPT_DRIVER_HEADERS max2771_evkit_signal_source_fpga.h)
endif()

if(ENABLE_DMA_PROXY)
    ###############################################
    # FPGA DMA source
    ###############################################
    list(APPEND OPT_DRIVER_SOURCES dma_signal_source_fpga.cc)
    list(APPEND OPT_DRIVER_HEADERS dma_signal_source_fpga.h)
endif()

if(ENABLE_FLEXIBAND AND TELEORBIT_FOUND)
    list(APPEND OPT_DRIVER_SOURCES flexiband_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS flexiband_signal_source.h)
endif()

if(ENABLE_ARRAY)
    ##############################################
    # DBFCTTC GNSS EXPERIMENTAL ARRAY PROTOTYPE
    ##############################################
    if(NOT GRDBFCTTC_FOUND)
        message(" gr-dbfcttc not found, install it from https://github.com/gnss-sdr/gr-dbfcttc")
        message(FATAL_ERROR "gr-dbfcttc required for building gnss-sdr with this option enabled")
    endif()
    list(APPEND OPT_DRIVER_SOURCES raw_array_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS raw_array_signal_source.h)
endif()

if(ENABLE_OSMOSDR)
    ################################################################################
    # OsmoSDR - https://osmocom.org/projects/gr-osmosdr/
    ################################################################################
    if(GROSMOSDR_FOUND)
        list(APPEND OPT_DRIVER_SOURCES osmosdr_signal_source.cc)
        list(APPEND OPT_DRIVER_HEADERS osmosdr_signal_source.h)
    endif()
endif()

if(ENABLE_LIMESDR)
    if(GRLIMESDR_FOUND)
        list(APPEND OPT_DRIVER_SOURCES limesdr_signal_source.cc)
        list(APPEND OPT_DRIVER_HEADERS limesdr_signal_source.h)
    endif()
endif()

if(ENABLE_UHD)
    list(APPEND OPT_DRIVER_SOURCES uhd_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS uhd_signal_source.h)
endif()

if(ENABLE_ZMQ)
    list(APPEND OPT_DRIVER_SOURCES zmq_signal_source.cc)
    list(APPEND OPT_DRIVER_HEADERS zmq_signal_source.h)
endif()

set(SIGNAL_SOURCE_ADAPTER_SOURCES
    signal_source_base.cc
    file_source_base.cc
    file_signal_source.cc
    fifo_signal_source.cc
    multichannel_file_signal_source.cc
    gen_signal_source.cc
    nsr_file_signal_source.cc
    spir_file_signal_source.cc
    spir_gss6450_file_signal_source.cc
    rtl_tcp_signal_source.cc
    labsat_signal_source.cc
    two_bit_cpx_file_signal_source.cc
    two_bit_packed_file_signal_source.cc
    four_bit_cpx_file_signal_source.cc
    file_timestamp_signal_source.cc
    ${OPT_DRIVER_SOURCES}
)

set(SIGNAL_SOURCE_ADAPTER_HEADERS
    signal_source_base.h
    file_source_base.h
    file_signal_source.h
    fifo_signal_source.h
    multichannel_file_signal_source.h
    gen_signal_source.h
    nsr_file_signal_source.h
    spir_file_signal_source.h
    spir_gss6450_file_signal_source.h
    rtl_tcp_signal_source.h
    labsat_signal_source.h
    two_bit_cpx_file_signal_source.h
    two_bit_packed_file_signal_source.h
    four_bit_cpx_file_signal_source.h
    file_timestamp_signal_source.h
    ${OPT_DRIVER_HEADERS}
)

list(SORT SIGNAL_SOURCE_ADAPTER_HEADERS)
list(SORT SIGNAL_SOURCE_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(signal_source_adapters STATIC)
    target_sources(signal_source_adapters
        PRIVATE
            ${SIGNAL_SOURCE_ADAPTER_SOURCES}
        PUBLIC
            ${SIGNAL_SOURCE_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${SIGNAL_SOURCE_ADAPTER_HEADERS})
    add_library(signal_source_adapters
        ${SIGNAL_SOURCE_ADAPTER_SOURCES}
        ${SIGNAL_SOURCE_ADAPTER_HEADERS}
    )
endif()

if(GNURADIO_IS_38_OR_GREATER)
    target_compile_definitions(signal_source_adapters PUBLIC -DGR_GREATER_38=1)
endif()

target_include_directories(signal_source_adapters
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

if(ENABLE_FPGA OR ENABLE_AD9361)
    target_link_libraries(signal_source_adapters
        PUBLIC
            signal_source_libs
        PRIVATE
            core_libs
    )
endif()

target_link_libraries(signal_source_adapters
    PUBLIC
        Boost::headers
        Gnuradio::blocks
        signal_source_gr_blocks
    PRIVATE
        algorithms_libs
        gnss_sdr_flags
        core_system_parameters
        Volk::volk
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(signal_source_adapters PRIVATE Glog::glog)
    target_compile_definitions(signal_source_adapters PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(signal_source_adapters PRIVATE absl::flags absl::log)
endif()

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_source_adapters
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(ENABLE_RAW_UDP AND PCAP_FOUND)
    target_link_libraries(signal_source_adapters
        PRIVATE
            Pcap::pcap
    )
endif()

if(ENABLE_UHD)
    target_link_libraries(signal_source_adapters
        PUBLIC
            Gnuradio::uhd
            Uhd::uhd  # for uhd/config.hpp in gnuradio/uhd/usrp_source.h
    )
endif()

if(ENABLE_ZMQ)
    target_link_libraries(signal_source_adapters
        PUBLIC
            Gnuradio::zeromq
    )
endif()

if(ENABLE_OSMOSDR AND GROSMOSDR_FOUND)
    target_link_libraries(signal_source_adapters
        PUBLIC
            Gnuradio::osmosdr
    )
endif()

if(ENABLE_LIMESDR AND GRLIMESDR_FOUND)
    target_link_libraries(signal_source_adapters
        PUBLIC
            Gnuradio::limesdr
    )
endif()

if(ENABLE_AD9361 AND LIBIIO_FOUND)
    target_link_libraries(signal_source_adapters
        PRIVATE
            Iio::iio
    )
endif()

if(ENABLE_AD9361 OR ENABLE_FMCOMMS2 OR ENABLE_PLUTOSDR)
    if(LIBAD9361_VERSION)
        if(LIBAD9361_VERSION VERSION_GREATER 0.1)
            target_compile_definitions(signal_source_adapters
                PRIVATE -DLIBAD9361_VERSION_GREATER_THAN_01=1
            )
        endif()
    endif()
endif()

if(ENABLE_FLEXIBAND AND TELEORBIT_FOUND)
    target_link_libraries(signal_source_adapters
        PRIVATE
            Gnuradio::teleorbit
    )
endif()

if(ENABLE_ARRAY AND GRDBFCTTC_FOUND)
    target_link_libraries(signal_source_adapters
        PRIVATE
            Gnuradio::dbfcttc
    )
endif()

if(ENABLE_PLUTOSDR OR ENABLE_FMCOMMS2)
    if(GR_IIO_INCLUDE_HAS_GNURADIO)
        target_compile_definitions(signal_source_adapters
            PUBLIC -DGRIIO_INCLUDE_HAS_GNURADIO=1
        )
    endif()
    if(GNURADIO_API_IIO)
        target_compile_definitions(signal_source_adapters
            PUBLIC -DGNURADIO_API_IIO=1
        )
    endif()
    target_link_libraries(signal_source_adapters
        PUBLIC
            Gnuradio::iio
            Boost::chrono
        PRIVATE
            Iio::iio
    )
    if(NOT GNURADIO_API_IIO)
        target_link_libraries(signal_source_adapters
            PRIVATE
                Gnuradio::analog
        )
    endif()
    if(GR_IIO_TEMPLATIZED_API)
        target_compile_definitions(signal_source_adapters
            PUBLIC -DGR_IIO_TEMPLATIZED_API=1
        )
    endif()
endif()

if(GR_LIMESDR_IS_G38_BRANCH)
    target_compile_definitions(signal_source_adapters PRIVATE -DGR_LIMESDR_IS_G38_BRANCH=1)
endif()

if(GRLIMESDR_PPS)
    target_compile_definitions(signal_source_adapters PRIVATE -DLimeSDR_PPS=1)
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_source_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

target_compile_definitions(signal_source_adapters
    PRIVATE -DGNSSSDR_INSTALL_DIR="${CMAKE_INSTALL_PREFIX}"
)

set_property(TARGET signal_source_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
