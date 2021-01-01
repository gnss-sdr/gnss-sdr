# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(DATATYPE_ADAPTER_SOURCES
    byte_to_short.cc
    ibyte_to_cbyte.cc
    ibyte_to_complex.cc
    ibyte_to_cshort.cc
    ishort_to_cshort.cc
    ishort_to_complex.cc
)

set(DATATYPE_ADAPTER_HEADERS
    byte_to_short.h
    ibyte_to_cbyte.h
    ibyte_to_complex.h
    ibyte_to_cshort.h
    ishort_to_cshort.h
    ishort_to_complex.h
)

list(SORT DATATYPE_ADAPTER_HEADERS)
list(SORT DATATYPE_ADAPTER_SOURCES)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(data_type_adapters STATIC)
    target_sources(data_type_adapters
        PRIVATE
            ${DATATYPE_ADAPTER_SOURCES}
        PUBLIC
            ${DATATYPE_ADAPTER_HEADERS}
    )
else()
    source_group(Headers FILES ${DATATYPE_ADAPTER_HEADERS})
    add_library(data_type_adapters
        ${DATATYPE_ADAPTER_SOURCES}
        ${DATATYPE_ADAPTER_HEADERS}
    )
endif()

target_include_directories(data_type_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

target_link_libraries(data_type_adapters
    PUBLIC
        algorithms_libs
        Gnuradio::blocks
        data_type_gr_blocks
    PRIVATE
        Gflags::gflags
        Glog::glog
        Volk::volk
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(data_type_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET data_type_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
