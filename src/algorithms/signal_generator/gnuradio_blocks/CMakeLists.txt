# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
    add_library(signal_generator_gr_blocks STATIC)
    target_sources(signal_generator_gr_blocks
        PRIVATE
            signal_generator_c.cc
        PUBLIC
            signal_generator_c.h
    )
else()
    source_group(Headers FILES signal_generator_c.h)
    add_library(signal_generator_gr_blocks
        signal_generator_c.cc
        signal_generator_c.h
    )
endif()

target_link_libraries(signal_generator_gr_blocks
    PUBLIC
        Gnuradio::runtime
    PRIVATE
        algorithms_libs
        core_system_parameters
        Volkgnsssdr::volkgnsssdr
)

if(GNURADIO_USES_STD_POINTERS)
    target_compile_definitions(signal_generator_gr_blocks
        PUBLIC -DGNURADIO_USES_STD_POINTERS=1
    )
endif()

target_include_directories(signal_generator_gr_blocks
    PUBLIC
        ${GNSSSDR_SOURCE_DIR}/src/core/interfaces
)

# Workaround for CentOS 7
if((CMAKE_CXX_COMPILER_ID STREQUAL "GNU") AND
    (CMAKE_CXX_COMPILER_VERSION VERSION_LESS "5.0"))
    target_compile_options(signal_generator_gr_blocks
        PRIVATE
            $<$<CONFIG:Release>:-O0>
            $<$<CONFIG:RelWithDebInfo>:-O0>
    )
endif()

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(signal_generator_gr_blocks
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET signal_generator_gr_blocks
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
