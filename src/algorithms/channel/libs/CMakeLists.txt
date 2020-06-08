# Copyright (C) 2012-2020  (see AUTHORS file for a list of contributors)
#
# GNSS-SDR is a software-defined Global Navigation Satellite Systems receiver
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

set(CHANNEL_FSM_SOURCES
    channel_fsm.cc
    channel_msg_receiver_cc.cc
)

set(CHANNEL_FSM_HEADERS
    channel_fsm.h
    channel_msg_receiver_cc.h
)

list(SORT CHANNEL_FSM_HEADERS)
list(SORT CHANNEL_FSM_SOURCES)

if(CMAKE_VERSION VERSION_GREATER 3.13)
    add_library(channel_libs STATIC)
    target_sources(channel_libs
        PRIVATE
            ${CHANNEL_FSM_SOURCES}
        PUBLIC
            ${CHANNEL_FSM_HEADERS}
    )
else()
    source_group(Headers FILES ${CHANNEL_FSM_HEADERS})
    add_library(channel_libs ${CHANNEL_FSM_SOURCES} ${CHANNEL_FSM_HEADERS})
endif()

target_link_libraries(channel_libs
    PUBLIC
        core_system_parameters
        Gnuradio::runtime
        Gnuradio::pmt
    PRIVATE
        core_libs
        Boost::headers
        Gflags::gflags
        Glog::glog
)

target_include_directories(channel_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(channel_libs
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

if(CMAKE_VERSION VERSION_GREATER 3.1 AND NOT ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)))
    set(has_generic_lambdas HAS_GENERIC_LAMBDA=1)
    set(no_has_generic_lambdas HAS_GENERIC_LAMBDA=0)
    target_compile_definitions(channel_libs
        PRIVATE
            "$<$<COMPILE_FEATURES:cxx_generic_lambdas>:${has_generic_lambdas}>"
            "$<$<NOT:$<COMPILE_FEATURES:cxx_generic_lambdas>>:${no_has_generic_lambdas}>"
    )
else()
    target_compile_definitions(channel_libs
        PRIVATE
            -DHAS_GENERIC_LAMBDA=0
    )
endif()

if(Boost_VERSION_STRING VERSION_GREATER 1.72.99)
    target_compile_definitions(channel_libs
        PRIVATE
            -DBOOST_173_OR_GREATER=1
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(channel_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET channel_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
