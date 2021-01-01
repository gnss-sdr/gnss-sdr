# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


set(SYSTEM_TESTING_LIB_SOURCES
    spirent_motion_csv_dump_reader.cc
    rtklib_solver_dump_reader.cc
)

file(GLOB SYSTEM_TESTING_LIB_HEADERS "*.h")
list(SORT SYSTEM_TESTING_LIB_HEADERS)

if(USE_CMAKE_TARGET_SOURCES)
    add_library(system_testing_lib STATIC)
    target_sources(system_testing_lib
        PRIVATE
            ${SYSTEM_TESTING_LIB_SOURCES}
        PUBLIC
            ${SYSTEM_TESTING_LIB_HEADERS}
    )
else()
    source_group(Headers FILES ${SYSTEM_TESTING_LIB_HEADERS})
    add_library(system_testing_lib
        ${SYSTEM_TESTING_LIB_SOURCES}
        ${SYSTEM_TESTING_LIB_HEADERS}
    )
endif()

target_link_libraries(system_testing_lib
    PUBLIC
        Armadillo::armadillo
        Gflags::gflags
    PRIVATE
        Boost::headers
        Matio::matio
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(system_testing_lib
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET system_testing_lib
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
