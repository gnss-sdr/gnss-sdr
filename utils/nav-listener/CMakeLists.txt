# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2021 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.9...4.0)
project(nav-msg-listener CXX)

set(CMAKE_CXX_STANDARD 11)

set(NAVLISTENER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}) # allows this to be a sub-project
set(NAVLISTENER_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    # Workaround for Macports
    if(NOT BOOST_ROOT)
        set(MACOS_PACKAGES_PREFIX "")
        # Detect if MacPorts is installed on this system; if so, return base path and version
        execute_process(COMMAND which port RESULT_VARIABLE DETECT_MACPORTS OUTPUT_VARIABLE MACPORTS_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
        if(${DETECT_MACPORTS} EQUAL 0)
            # "/opt/local/bin/port", so we get the parent directory
            get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
            # "/opt/local/bin", so we get the parent directory
            get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
            execute_process(COMMAND port version RESULT_VARIABLE DETECT_MACPORTS_VERSION OUTPUT_VARIABLE MACPORTS_VERSION ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
            string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" MACPORTS_VERSION "${MACPORTS_VERSION}")
            set(MACOS_PACKAGES_PREFIX ${MACPORTS_PREFIX})
        endif()
        if(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.88")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.88")
        elseif(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.87")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.87")
        elseif(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.81")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.81")
        elseif(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.78")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.78")
        elseif(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.76")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.76")
        elseif(EXISTS "${MACPORTS_PREFIX}/libexec/boost/1.71")
            set(BOOST_ROOT "${MACPORTS_PREFIX}/libexec/boost/1.71")
        endif()
    endif()
endif()

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost COMPONENTS system)

if(Boost_VERSION_STRING VERSION_GREATER "1.88.99")
    set(NEED_BOOST_SYSTEM OFF)
else()
    set(NEED_BOOST_SYSTEM ON)
endif()

find_package(Protobuf REQUIRED)
if(${Protobuf_VERSION} VERSION_LESS "3.0.0")
    message(FATAL_ERROR "Fatal error: Protocol Buffers >= v3.0.0 required.")
endif()

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${NAVLISTENER_SOURCE_DIR}/nav_message.proto)

add_library(navmsg_lib ${NAVLISTENER_SOURCE_DIR}/nav_msg_udp_listener.cc ${PROTO_SRCS})

target_link_libraries(navmsg_lib
    PUBLIC
        Boost::headers
        $<$<BOOL:${NEED_BOOST_SYSTEM}>:Boost::system>
        protobuf::libprotobuf
)

target_include_directories(navmsg_lib
    PUBLIC
        ${NAVLISTENER_BINARY_DIR}
)

add_executable(nav_msg_listener ${NAVLISTENER_SOURCE_DIR}/main.cc)

target_link_libraries(nav_msg_listener PUBLIC navmsg_lib)

install(TARGETS nav_msg_listener
    RUNTIME DESTINATION bin
    COMPONENT "nav_msg_listener"
)

if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY
    )
    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    )
endif()
