# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_SOURCE_DIR}/docs/protobuf/gnss_synchro.proto)

set(CORE_MONITOR_LIBS_SOURCES
    gnss_synchro_monitor.cc
    gnss_synchro_udp_sink.cc
)

set(CORE_MONITOR_LIBS_HEADERS
    gnss_synchro_monitor.h
    gnss_synchro_udp_sink.h
    serdes_gnss_synchro.h
)

list(SORT CORE_MONITOR_LIBS_HEADERS)
list(SORT CORE_MONITOR_LIBS_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(core_monitor STATIC)
    target_sources(core_monitor
        PRIVATE
            ${PROTO_SRCS}
            ${PROTO_HDRS}
            ${CORE_MONITOR_LIBS_SOURCES}
        PUBLIC
            ${CORE_MONITOR_LIBS_HEADERS}
    )
else()
    source_group(Headers FILES ${CORE_MONITOR_LIBS_HEADERS})
    add_library(core_monitor
        ${CORE_MONITOR_LIBS_SOURCES}
        ${PROTO_SRCS}
        ${CORE_MONITOR_LIBS_HEADERS}
        ${PROTO_HDRS}
    )
endif()

target_link_libraries(core_monitor
    PUBLIC
        Boost::headers
        Boost::system
        Gnuradio::runtime
        protobuf::libprotobuf
        core_system_parameters
    PRIVATE
        Boost::serialization
        Gnuradio::pmt
)

get_filename_component(PROTO_INCLUDE_HEADERS ${PROTO_HDRS} DIRECTORY)

target_include_directories(core_monitor
    PUBLIC
        SYSTEM ${PROTO_INCLUDE_HEADERS}
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(core_monitor
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(USE_BOOST_ASIO_IO_CONTEXT)
    target_compile_definitions(core_monitor
        PUBLIC
            -DUSE_BOOST_ASIO_IO_CONTEXT=1
    )
endif()

# Fix for Boost Asio < 1.70
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if((CMAKE_CXX_COMPILER_ID MATCHES "Clang") AND (Boost_VERSION_STRING VERSION_LESS 1.70.0))
        if(${has_string_view})
            target_compile_definitions(core_monitor
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=1
            )
        else()
            target_compile_definitions(core_monitor
                PUBLIC
                    -DBOOST_ASIO_HAS_STD_STRING_VIEW=0
            )
        endif()
    endif()
endif()

set_property(TARGET core_monitor
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
