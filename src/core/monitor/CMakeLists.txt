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

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_SOURCE_DIR}/docs/protobuf/gnss_synchro.proto)

set(CORE_MONITOR_LIBS_SOURCES
    gnss_synchro_monitor.cc
    gnss_synchro_udp_sink.cc
    ${PROTO_SRCS}
)

set(CORE_MONITOR_LIBS_HEADERS
    gnss_synchro_monitor.h
    gnss_synchro_udp_sink.h
    serdes_gnss_synchro.h
    ${PROTO_HDRS}
)

list(SORT CORE_MONITOR_LIBS_HEADERS)
list(SORT CORE_MONITOR_LIBS_SOURCES)

source_group(Headers FILES ${CORE_MONITOR_LIBS_HEADERS})

add_library(core_monitor
    ${CORE_MONITOR_LIBS_SOURCES}
    ${CORE_MONITOR_LIBS_HEADERS}
)

target_link_libraries(core_monitor
    PUBLIC
        Boost::serialization
        Boost::system
        Gnuradio::runtime
        protobuf::libprotobuf
        core_system_parameters
    PRIVATE
        Gnuradio::pmt
)

get_filename_component(PROTO_INCLUDE_HEADERS ${PROTO_HDRS} DIRECTORY)

target_include_directories(core_monitor
    PUBLIC
        ${PROTO_INCLUDE_HEADERS}
)


if(Boost_VERSION_STRING VERSION_GREATER 1.65.99)
    target_compile_definitions(core_monitor
        PUBLIC
            -DBOOST_GREATER_1_65
    )
endif()


# Fix for Boost Asio < 1.70
if(OS_IS_MACOSX)
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


if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(core_monitor
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()


set_property(TARGET core_monitor
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${PROTO_INCLUDE_HEADERS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
