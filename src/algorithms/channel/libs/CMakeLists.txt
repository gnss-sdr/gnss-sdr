# Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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

include_directories(
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/src/core/system_parameters
     ${CMAKE_SOURCE_DIR}/src/core/interfaces
     ${CMAKE_SOURCE_DIR}/src/core/receiver
     ${CMAKE_SOURCE_DIR}/src/algorithms/channel/adapters
     ${Boost_INCLUDE_DIRS}
     ${GLOG_INCLUDE_DIRS}
     ${GFlags_INCLUDE_DIRS}
     ${GNURADIO_RUNTIME_INCLUDE_DIRS}
)

list(SORT CHANNEL_FSM_HEADERS)
list(SORT CHANNEL_FSM_SOURCES)

add_library(channel_fsm ${CHANNEL_FSM_SOURCES} ${CHANNEL_FSM_HEADERS})
source_group(Headers FILES ${CHANNEL_FSM_HEADERS})
add_dependencies(channel_fsm glog-${glog_RELEASE})

target_link_libraries(channel_fsm gnss_rx)
