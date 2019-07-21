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

source_group(Headers FILES ${CHANNEL_FSM_HEADERS})

add_library(channel_libs ${CHANNEL_FSM_SOURCES} ${CHANNEL_FSM_HEADERS})

target_link_libraries(channel_libs
    PUBLIC
        core_system_parameters
        Gnuradio::runtime
        Gnuradio::pmt
    PRIVATE
        core_libs
        Boost::boost
        Gflags::gflags
        Glog::glog
)

target_include_directories(channel_libs
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/receiver
)

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
