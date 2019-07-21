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

add_subdirectory(supl)

set(CORE_LIBS_SOURCES
    ini.cc
    INIReader.cc
    string_converter.cc
    gnss_sdr_supl_client.cc
    gnss_sdr_sample_counter.cc
    channel_status_msg_receiver.cc
    channel_event.cc
    command_event.cc
)

set(CORE_LIBS_HEADERS
    ini.h
    INIReader.h
    string_converter.h
    gnss_sdr_supl_client.h
    gnss_sdr_sample_counter.h
    channel_status_msg_receiver.h
    channel_event.h
    command_event.h
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

list(SORT CORE_LIBS_HEADERS)
list(SORT CORE_LIBS_SOURCES)

source_group(Headers FILES ${CORE_LIBS_HEADERS})

add_library(core_libs ${CORE_LIBS_SOURCES} ${CORE_LIBS_HEADERS})

target_link_libraries(core_libs
    PUBLIC
        Boost::boost
        Gnuradio::runtime
        core_libs_supl
        core_system_parameters
        pvt_libs
    PRIVATE
        Boost::serialization
        Gflags::gflags
        Glog::glog
        Pugixml::pugixml
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
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/core/interfaces>
)
