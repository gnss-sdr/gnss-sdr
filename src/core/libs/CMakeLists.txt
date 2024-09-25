# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${GNSSSDR_SOURCE_DIR}/docs/protobuf/nav_message.proto)

add_subdirectory(supl)

set(CORE_LIBS_SOURCES
    channel_event.cc
    channel_status_msg_receiver.cc
    command_event.cc
    galileo_e6_has_msg_receiver.cc
    galileo_tow_map.cc
    gnss_crypto.cc
    gnss_sdr_sample_counter.cc
    gnss_sdr_supl_client.cc
    ini.cc
    INIReader.cc
    nav_message_monitor.cc
    nav_message_udp_sink.cc
    osnma_helper.cc
    osnma_msg_receiver.cc
    osnma_nav_data_manager.cc
    string_converter.cc
)

set(CORE_LIBS_HEADERS
    channel_event.h
    channel_status_msg_receiver.h
    command_event.h
    galileo_tow_map.h
    gnss_crypto.h
    gnss_sdr_sample_counter.h
    gnss_sdr_supl_client.h
    ini.h
    INIReader.h
    nav_message_monitor.h
    nav_message_packet.h
    nav_message_udp_sink.h
    osnma_helper.h
    osnma_msg_receiver.h
    osnma_nav_data_manager.h
    serdes_nav_message.h
    string_converter.h
)

if(ENABLE_FPGA)
    set(CORE_LIBS_SOURCES
        ${CORE_LIBS_SOURCES}
        gnss_sdr_fpga_sample_counter.cc
        gnss_sdr_time_counter.cc
    )
    set(CORE_LIBS_HEADERS
        ${CORE_LIBS_HEADERS}
        gnss_sdr_fpga_sample_counter.h
        gnss_sdr_time_counter.h
    )
endif()

if(ENABLE_FPGA OR ENABLE_AD9361)
    set(CORE_LIBS_SOURCES
        ${CORE_LIBS_SOURCES}
        uio_fpga.cc
    )
    set(CORE_LIBS_HEADERS
        ${CORE_LIBS_HEADERS}
        uio_fpga.h
    )
endif()

list(SORT CORE_LIBS_HEADERS)
list(SORT CORE_LIBS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_libs STATIC)
    target_sources(core_libs
        PRIVATE
            ${CORE_LIBS_SOURCES}
            ${PROTO_SRCS}
            ${PROTO_HDRS}
        PUBLIC
            ${CORE_LIBS_HEADERS}
    )
else()
    source_group(Headers FILES ${CORE_LIBS_HEADERS} ${PROTO_HDRS})
    add_library(core_libs ${CORE_LIBS_SOURCES} ${CORE_LIBS_HEADERS} ${PROTO_SRCS} ${PROTO_HDRS})
endif()

target_link_libraries(core_libs
    PUBLIC
        Boost::headers
        Gnuradio::blocks
        Gnuradio::runtime
        Gnuradio::pmt
        protobuf::libprotobuf
        core_libs_supl
        core_system_parameters
        pvt_libs
    PRIVATE
        algorithms_libs
        Boost::serialization
        Boost::system
        Pugixml::pugixml
)

if(ENABLE_GLOG_AND_GFLAGS)
    target_link_libraries(core_libs PRIVATE Gflags::gflags Glog::glog)
    target_compile_definitions(core_libs PRIVATE -DUSE_GLOG_AND_GFLAGS=1)
else()
    target_link_libraries(core_libs PRIVATE absl::flags absl::log)
endif()

if(USE_GENERIC_LAMBDAS AND NOT GNURADIO_USES_STD_POINTERS)
    target_link_libraries(core_libs PUBLIC Boost::headers)
else()
    target_link_libraries(core_libs PRIVATE Boost::headers)
endif()

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(core_libs
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_include_directories(core_libs
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

# links to the appropriate library and defines
# USE_GNUTLS_FALLBACK, USE_OPENSSL_3, or USE_OPENSSL_111 accordingly.
link_to_crypto_dependencies(core_libs)

if(USE_GENERIC_LAMBDAS)
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(core_libs
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(core_libs
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(USE_BOOST_BIND_PLACEHOLDERS)
    target_compile_definitions(core_libs
        PRIVATE
            -DUSE_BOOST_BIND_PLACEHOLDERS=1
    )
endif()

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(core_libs
        PUBLIC
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

if(PMT_USES_BOOST_ANY)
    target_compile_definitions(core_libs
        PRIVATE
            -DPMT_USES_BOOST_ANY=1
    )
endif()

# Do not apply clang-tidy fixes to protobuf generated headers
get_filename_component(PROTO_INCLUDE_HEADERS_DIR ${PROTO_HDRS} DIRECTORY)
target_include_directories(core_libs
    SYSTEM PUBLIC
        ${PROTO_INCLUDE_HEADERS_DIR}
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(core_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET core_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${GNSSSDR_SOURCE_DIR}/src/core/interfaces>
)
