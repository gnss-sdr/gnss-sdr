# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2022 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(ACQUISITION_LIB_HEADERS acq_conf.h)
set(ACQUISITION_LIB_SOURCES acq_conf.cc)

if(ENABLE_FPGA)
    set(ACQUISITION_LIB_SOURCES ${ACQUISITION_LIB_SOURCES} acq_conf_fpga.cc)
    set(ACQUISITION_LIB_HEADERS ${ACQUISITION_LIB_HEADERS} acq_conf_fpga.h)
    set(ACQUISITION_LIB_SOURCES ${ACQUISITION_LIB_SOURCES} fpga_acquisition.cc)
    set(ACQUISITION_LIB_HEADERS ${ACQUISITION_LIB_HEADERS} fpga_acquisition.h)
endif()

list(SORT ACQUISITION_LIB_HEADERS)
list(SORT ACQUISITION_LIB_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(acquisition_libs STATIC)
    target_sources(acquisition_libs
        PRIVATE
            ${ACQUISITION_LIB_SOURCES}
        PUBLIC
            ${ACQUISITION_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${ACQUISITION_LIB_HEADERS})
    add_library(acquisition_libs
        ${ACQUISITION_LIB_SOURCES}
        ${ACQUISITION_LIB_HEADERS}
    )
endif()

target_link_libraries(acquisition_libs
    INTERFACE
        Gnuradio::runtime
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
        $<BUILD_INTERFACE:${GNSSSDR_SOURCE_DIR}/src/core/interfaces>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

if(ENABLE_FPGA)
    target_link_libraries(acquisition_libs
        PRIVATE
            core_libs
    )
endif()

