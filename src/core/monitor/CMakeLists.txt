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


set(CORE_MONITOR_LIBS_SOURCES
    gnss_synchro_monitor.cc
    gnss_synchro_udp_sink.cc
)

set(CORE_MONITOR_LIBS_HEADERS
    gnss_synchro_monitor.h
    gnss_synchro_udp_sink.h
)

list(SORT CORE_MONITOR_LIBS_HEADERS)
list(SORT CORE_MONITOR_LIBS_SOURCES)

add_library(core_monitor_lib
    ${CORE_MONITOR_LIBS_SOURCES}
    ${CORE_MONITOR_LIBS_HEADERS}
)

source_group(Headers FILES ${CORE_MONITOR_LIBS_HEADERS})

target_link_libraries(core_monitor_lib
    PUBLIC
        Boost::serialization
        Boost::system
        gnss_system_parameters
        Gnuradio::runtime
    PRIVATE
        Gnuradio::pmt
        Gflags::gflags
        Glog::glog
)

set_property(TARGET core_monitor_lib
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
