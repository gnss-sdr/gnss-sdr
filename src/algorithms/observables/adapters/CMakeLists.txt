# GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
# This file is part of GNSS-SDR.
#
# SPDX-FileCopyrightText: 2010-2020 C. Fernandez-Prades cfernandez(at)cttc.es
# SPDX-License-Identifier: BSD-3-Clause


if(USE_CMAKE_TARGET_SOURCES)
    add_library(obs_adapters STATIC)
    target_sources(obs_adapters
        PRIVATE
            hybrid_observables.cc
        PUBLIC
            hybrid_observables.h
    )
else()
    source_group(Headers FILES hybrid_observables.h)
    add_library(obs_adapters hybrid_observables.cc hybrid_observables.h)
endif()

target_include_directories(obs_adapters
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src/core/interfaces
)

target_link_libraries(obs_adapters
    PUBLIC
        obs_gr_blocks
        core_system_parameters
    PRIVATE
        gnss_sdr_flags
        Glog::glog
)

if(ENABLE_CLANG_TIDY)
    if(CLANG_TIDY_EXE)
        set_target_properties(obs_adapters
            PROPERTIES
                CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
        )
    endif()
endif()

set_property(TARGET obs_adapters
    APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
