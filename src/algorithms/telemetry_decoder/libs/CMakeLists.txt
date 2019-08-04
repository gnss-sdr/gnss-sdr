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


add_subdirectory(libswiftcnav)

set(TELEMETRY_DECODER_LIB_SOURCES
    viterbi_decoder.cc
)

set(TELEMETRY_DECODER_LIB_HEADERS
    viterbi_decoder.h
    convolutional.h
)

list(SORT TELEMETRY_DECODER_LIB_HEADERS)
list(SORT TELEMETRY_DECODER_LIB_SOURCES)

source_group(Headers FILES ${TELEMETRY_DECODER_LIB_HEADERS})

add_library(telemetry_decoder_libs
    ${TELEMETRY_DECODER_LIB_SOURCES}
    ${TELEMETRY_DECODER_LIB_HEADERS}
)

target_link_libraries(telemetry_decoder_libs
    PRIVATE
        Gflags::gflags
        Glog::glog
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(telemetry_decoder_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET telemetry_decoder_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
