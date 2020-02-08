# Copyright (C) 2012-2019  (see AUTHORS file for a list of contributors)
#
# This file is part of GNSS-SDR.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

if(ENABLE_FPGA)
    set(ACQUISITION_LIB_SOURCES fpga_acquisition.cc)
    set(ACQUISITION_LIB_HEADERS fpga_acquisition.h)
endif()

set(ACQUISITION_LIB_HEADERS ${ACQUISITION_LIB_HEADERS} acq_conf.h)
set(ACQUISITION_LIB_SOURCES ${ACQUISITION_LIB_SOURCES} acq_conf.cc)

list(SORT ACQUISITION_LIB_HEADERS)
list(SORT ACQUISITION_LIB_SOURCES)

source_group(Headers FILES ${ACQUISITION_LIB_HEADERS})

add_library(acquisition_libs
    ${ACQUISITION_LIB_SOURCES}
    ${ACQUISITION_LIB_HEADERS}
)

target_link_libraries(acquisition_libs
    PUBLIC
        Volk::volk
    PRIVATE
        Gflags::gflags
        Glog::glog
        algorithms_libs
        core_system_parameters
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(acquisition_libs
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET acquisition_libs
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
