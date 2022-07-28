# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.9...3.23)
project(nav-msg-listener CXX)

set(CMAKE_CXX_STANDARD 11)

set(NAVLISTENER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # allows this to be a sub-project
set(NAVLISTENER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS system REQUIRED)

find_package(Protobuf REQUIRED)
if(${Protobuf_VERSION} VERSION_LESS "3.0.0")
    message(FATAL_ERROR "Fatal error: Protocol Buffers >= v3.0.0 required.")
endif()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${NAVLISTENER_SOURCE_DIR}/nav_message.proto)

add_library(navmsg_lib ${NAVLISTENER_SOURCE_DIR}/nav_msg_udp_listener.cc ${PROTO_SRCS})

target_link_libraries(navmsg_lib
    PUBLIC
        Boost::boost
        Boost::system
        protobuf::libprotobuf
)

target_include_directories(navmsg_lib
    PUBLIC
        ${NAVLISTENER_BINARY_DIR}
)

add_executable(nav_msg_listener ${NAVLISTENER_SOURCE_DIR}/main.cc)

target_link_libraries(nav_msg_listener PUBLIC navmsg_lib)
