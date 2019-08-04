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


set(SIGNAL_PROCESSING_TESTING_LIB_SOURCES
    acquisition_dump_reader.cc
    acquisition_msg_rx.cc
    tracking_dump_reader.cc
    tlm_dump_reader.cc
    observables_dump_reader.cc
    tracking_true_obs_reader.cc
    true_observables_reader.cc
)

file(GLOB SIGNAL_PROCESSING_TESTING_LIB_HEADERS "*.h")
list(SORT SIGNAL_PROCESSING_TESTING_LIB_HEADERS)

add_library(signal_processing_testing_lib
    ${SIGNAL_PROCESSING_TESTING_LIB_SOURCES}
    ${SIGNAL_PROCESSING_TESTING_LIB_HEADERS}
)

source_group(Headers FILES ${SIGNAL_PROCESSING_TESTING_LIB_HEADERS})

target_link_libraries(signal_processing_testing_lib
    PUBLIC
        Armadillo::armadillo
        Gflags::gflags
        Gnuradio::runtime
    PRIVATE
        Boost::boost
        Matio::matio
        Glog::glog
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_processing_testing_lib
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_processing_testing_lib
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
