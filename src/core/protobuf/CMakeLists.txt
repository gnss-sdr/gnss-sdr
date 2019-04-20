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

add_library(core_protobuf serdes_gnss_synchro.h ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(core_protobuf
    PUBLIC
        protobuf::libprotobuf
)

target_include_directories(core_protobuf
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/system_parameters
)

get_filename_component(PROTO_INCLUDE_HEADERS ${PROTO_HDRS} DIRECTORY)
set_property(TARGET core_protobuf
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${PROTO_INCLUDE_HEADERS}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
